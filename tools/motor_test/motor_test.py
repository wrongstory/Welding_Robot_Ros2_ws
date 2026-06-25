#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L7NH EtherCAT(CoE) 모터 제어 테스트 — pysoem 마스터 (C++ motor_test.cpp 의 Python 미러).

대상  : LS Electric L7NH 서보 (CANopen over EtherCAT).
        cf. L7NH_200V_Manual_V1.4_EN §3.1, page 3-1.
인터페이스: EtherCAT 마스터 (예: eno1). raw socket → root 필요 (sudo 로 실행).
동작  : enable/disable, 상태/엔코더 읽기, Profile Velocity(mode=3) jog 회전.

CiA402 상태머신 로직 출처: servo_core/src/cia402.cpp (동일 DS402 상수).
PDO 매핑: RxPDO 0x1600 / TxPDO 0x1A00, 할당 0x1C12/0x1C13 (L7NH manual §3).

⚠ 안전: 실제 모터가 회전한다. 비상정지 수단·축 주변 안전 확인 후 enable/jog.
"""

import struct
import sys
import threading
import time

import pysoem

# ----- PDO 레이아웃 (C++ 버전 RxPdo/TxPdo 와 동일, little-endian, 패킹) -----
# Rx: controlword(u16) mode(i8) target_pos(i32) target_vel(i32) profile_vel(u32) = 15B
RX_FMT = "<HbiiI"
# Tx: statusword(u16) mode_disp(i8) pos_act(i32) vel_act(i32) torque(i16) err(u16) = 15B
TX_FMT = "<HbiihH"
assert struct.calcsize(RX_FMT) == 15
assert struct.calcsize(TX_FMT) == 15

# ----- CiA402 상수 (DS402 표준; servo_core/cia402.cpp 미러) -----
MASK_4F = 0x004F
MASK_6F = 0x006F
TARGET_REACHED_BIT = 1 << 10
FAULT_BIT = 1 << 3

CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE_OP = 0x000F
CW_QUICK_STOP = 0x0002
CW_FAULT_RESET = 0x0080

(NOT_READY, SWITCH_ON_DISABLED, READY_TO_SWITCH_ON, SWITCHED_ON,
 OPERATION_ENABLED, QUICK_STOP_ACTIVE, FAULT_REACTION, FAULT, UNKNOWN) = range(9)

STATE_NAME = {
    NOT_READY: "NotReadyToSwitchOn", SWITCH_ON_DISABLED: "SwitchOnDisabled",
    READY_TO_SWITCH_ON: "ReadyToSwitchOn", SWITCHED_ON: "SwitchedOn",
    OPERATION_ENABLED: "OperationEnabled", QUICK_STOP_ACTIVE: "QuickStopActive",
    FAULT_REACTION: "FaultReactionActive", FAULT: "Fault", UNKNOWN: "Unknown",
}


def decode_state(sw):
    if (sw & MASK_4F) == 0x0040:
        return SWITCH_ON_DISABLED
    if (sw & MASK_6F) == 0x0021:
        return READY_TO_SWITCH_ON
    if (sw & MASK_6F) == 0x0023:
        return SWITCHED_ON
    if (sw & MASK_6F) == 0x0027:
        return OPERATION_ENABLED
    if (sw & MASK_6F) == 0x0007:
        return QUICK_STOP_ACTIVE
    if (sw & MASK_4F) == 0x000F:
        return FAULT_REACTION
    if (sw & MASK_4F) == 0x0008:
        return FAULT
    if (sw & MASK_4F) == 0x0000:
        return NOT_READY
    return UNKNOWN


def controlword_to_enable(state):
    """현재 상태에서 OperationEnabled 로 한 단계 전진시키는 Controlword."""
    if state == FAULT:
        return CW_FAULT_RESET
    if state == SWITCH_ON_DISABLED:
        return CW_SHUTDOWN
    if state == READY_TO_SWITCH_ON:
        return CW_SWITCH_ON
    if state in (SWITCHED_ON, OPERATION_ENABLED):
        return CW_ENABLE_OP
    if state == QUICK_STOP_ACTIVE:
        return CW_SHUTDOWN
    return CW_SHUTDOWN  # 안전 기본값


def _u32_map(index, sub, bitlen):
    return (index << 16) | (sub << 8) | bitlen


class MotorTest:
    def __init__(self, ifname, jog_vel):
        self.ifname = ifname
        self.master = pysoem.Master()
        self.lock = threading.Lock()
        # 명령 (메인 → 주기 스레드)
        self.enable = False
        self.fault_reset = False
        self.direction = 0       # +1 / -1 / 0
        self.jog_vel = jog_vel   # 0x60FF Target Velocity (UU/s)
        # 상태 스냅샷 (주기 → 메인)
        self.tx = (0, 0, 0, 0, 0, 0)
        self.state = UNKNOWN
        self.cycle = 0
        self.running = False
        self._prev_reset = False

    # PreOP→SAFEOP 천이 중 호출되는 슬레이브 설정 훅: PDO 매핑 재구성.
    def _config_slave(self, pos):
        s = self.master.slaves[pos]

        def w8(idx, sub, v):
            s.sdo_write(idx, sub, struct.pack("<B", v))

        def w16(idx, sub, v):
            s.sdo_write(idx, sub, struct.pack("<H", v))

        def w32(idx, sub, v):
            s.sdo_write(idx, sub, struct.pack("<I", v))

        # 1) SM PDO 할당 해제
        w8(0x1C12, 0, 0)
        w8(0x1C13, 0, 0)
        # 2) RxPDO(0x1600) — RX_FMT 순서와 동일
        w8(0x1600, 0, 0)
        w32(0x1600, 1, _u32_map(0x6040, 0, 16))  # Controlword
        w32(0x1600, 2, _u32_map(0x6060, 0, 8))   # Modes of operation
        w32(0x1600, 3, _u32_map(0x607A, 0, 32))  # Target position
        w32(0x1600, 4, _u32_map(0x60FF, 0, 32))  # Target velocity
        w32(0x1600, 5, _u32_map(0x6081, 0, 32))  # Profile velocity
        w8(0x1600, 0, 5)
        # 3) TxPDO(0x1A00) — TX_FMT 순서와 동일
        w8(0x1A00, 0, 0)
        w32(0x1A00, 1, _u32_map(0x6041, 0, 16))  # Statusword
        w32(0x1A00, 2, _u32_map(0x6061, 0, 8))   # Modes display
        w32(0x1A00, 3, _u32_map(0x6064, 0, 32))  # Position actual
        w32(0x1A00, 4, _u32_map(0x606C, 0, 32))  # Velocity actual
        w32(0x1A00, 5, _u32_map(0x6077, 0, 16))  # Torque actual
        w32(0x1A00, 6, _u32_map(0x603F, 0, 16))  # Error code
        w8(0x1A00, 0, 6)
        # 4) SM 재할당
        w16(0x1C12, 1, 0x1600)
        w8(0x1C12, 0, 1)
        w16(0x1C13, 1, 0x1A00)
        w8(0x1C13, 0, 1)
        # 5) 기본 모드 PV(3) + 보수적 프로파일 (UU 스케일 현장 교정 필요)
        s.sdo_write(0x6060, 0, struct.pack("<b", 3))
        w32(0x6083, 0, 100000)
        w32(0x6084, 0, 100000)
        print("[setup] PDO 매핑 완료")

    def connect(self):
        self.master.open(self.ifname)
        if self.master.config_init() <= 0:
            raise RuntimeError("슬레이브를 찾지 못했습니다. EtherCAT 결선/전원 확인.")
        print("발견된 슬레이브: %d개 (slave1=%s)"
              % (len(self.master.slaves), self.master.slaves[0].name))
        self.master.slaves[0].config_func = self._config_slave
        self.master.config_map()
        if self.master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
            raise RuntimeError("SAFE_OP 천이 실패")
        # OP 천이 — process data 를 보내며 전환
        self.master.state = pysoem.OP_STATE
        self.master.send_processdata()
        self.master.receive_processdata(2000)
        self.master.write_state()
        for _ in range(200):
            self.master.send_processdata()
            self.master.receive_processdata(2000)
            if self.master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE:
                break
        if self.master.state != pysoem.OP_STATE:
            raise RuntimeError("OP 천이 실패")
        print("OP 상태 진입 — 주기 교환 시작.")

    def _cyclic(self):
        slave = self.master.slaves[0]
        period = 0.002
        while self.running:
            t_next = time.perf_counter() + period
            # 직전 TxPDO 해석
            raw_in = slave.input
            if raw_in and len(raw_in) >= 15:
                tx = struct.unpack(TX_FMT, bytes(raw_in[:15]))
            else:
                tx = (0, 0, 0, 0, 0, 0)
            st = decode_state(tx[0])

            with self.lock:
                enable = self.enable
                reset_req = self.fault_reset
                direction = self.direction
                jog_vel = self.jog_vel

            if reset_req:
                cw = CW_FAULT_RESET
                if self._prev_reset:
                    with self.lock:
                        self.fault_reset = False
            elif enable:
                cw = controlword_to_enable(st)
            else:
                cw = CW_SHUTDOWN
            self._prev_reset = reset_req

            can_move = (st == OPERATION_ENABLED) and enable
            target_vel = (jog_vel * direction) if (can_move and direction != 0) else 0
            rx = struct.pack(RX_FMT, cw, 3, tx[2], target_vel, 0)
            slave.output = rx

            self.master.send_processdata()
            self.master.receive_processdata(2000)

            with self.lock:
                self.tx = tx
                self.state = st
                self.cycle += 1

            dt = t_next - time.perf_counter()
            if dt > 0:
                time.sleep(dt)

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._cyclic, daemon=True)
        self._thread.start()

    def stop(self):
        # 안전 종료: 정지 → 서보 OFF → 주기 정지 → OP 해제
        with self.lock:
            self.direction = 0
            self.enable = False
        time.sleep(0.3)
        self.running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=1.0)
        self.master.state = pysoem.INIT_STATE
        self.master.write_state()
        self.master.close()

    def print_status(self):
        with self.lock:
            tx = self.tx
            st = self.state
            cyc = self.cycle
        sw, mode_disp, pos, vel, torque, err = tx
        flags = ""
        if sw & TARGET_REACHED_BIT:
            flags += "[target_reached]"
        if sw & FAULT_BIT:
            flags += "[FAULT]"
        print("[status] cyc=%d state=%s mode=%d pos=%d vel=%d torque=%d err=0x%04X %s"
              % (cyc, STATE_NAME.get(st, "?"), mode_disp, pos, vel, torque, err, flags))


MENU = """
==== L7NH EtherCAT 모터 테스트 (Python/pysoem) ====
 e: enable(서보 ON)   d: disable(서보 OFF)
 f: jog 정회전(+)     r: jog 역회전(-)     x: 정지
 v <n>: jog 속도 설정(0x60FF, UU/s)        c: fault reset
 p: 상태 1회 출력     q: 종료(감속·서보 OFF)
=================================================="""


def main():
    ifname = sys.argv[1] if len(sys.argv) > 1 else "eno1"
    jog_vel = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    print("⚠ 안전: 실제 모터가 회전합니다. 비상정지 수단·축 주변 안전을 확인하세요.")
    print("EtherCAT 인터페이스: %s,  초기 jog 속도: %d UU/s" % (ifname, jog_vel))

    mt = MotorTest(ifname, jog_vel)
    try:
        mt.connect()
    except Exception as exc:  # noqa: BLE001
        print("연결 실패: %s" % exc, file=sys.stderr)
        print("  - root 권한(sudo)으로 실행했는지, 인터페이스명/결선이 맞는지 확인.", file=sys.stderr)
        return 1

    mt.start()
    print(MENU)
    try:
        while True:
            try:
                line = input("cmd> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            c = parts[0]
            if c == "q":
                break
            elif c == "e":
                with mt.lock:
                    mt.enable = True
                print("enable 요청")
            elif c == "d":
                with mt.lock:
                    mt.enable = False
                    mt.direction = 0
                print("disable 요청")
            elif c == "f":
                with mt.lock:
                    mt.direction = 1
                print("jog 정회전 (vel=%d UU/s)" % mt.jog_vel)
            elif c == "r":
                with mt.lock:
                    mt.direction = -1
                print("jog 역회전 (vel=%d UU/s)" % mt.jog_vel)
            elif c == "x":
                with mt.lock:
                    mt.direction = 0
                print("정지(target velocity 0)")
            elif c == "v":
                if len(parts) >= 2 and parts[1].isdigit():
                    with mt.lock:
                        mt.jog_vel = int(parts[1])
                    print("jog 속도 = %d UU/s" % int(parts[1]))
                else:
                    print("사용법: v <0 이상 정수>")
            elif c == "c":
                with mt.lock:
                    mt.fault_reset = True
                print("fault reset 요청")
            elif c == "p":
                mt.print_status()
            else:
                print("알 수 없는 명령")
    except KeyboardInterrupt:
        pass
    finally:
        print("\n종료 중: 감속·서보 OFF...")
        mt.stop()
        print("종료 완료.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
