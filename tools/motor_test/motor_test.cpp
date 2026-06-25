// L7NH EtherCAT(CoE) 모터 제어 테스트 — SOEM 마스터.
//
// 대상  : LS Electric L7NH 서보 드라이브 (CANopen over EtherCAT).
//         cf. L7NH_200V_Manual_V1.4_EN §3.1 "Structure of CANopen over EtherCAT", page 3-1.
// 인터페이스: EtherCAT 마스터 (예: eno1). raw socket → root 권한 필요.
// 동작  : enable/disable, 상태/엔코더 읽기, Profile Velocity(mode=3) jog 회전.
//
// CiA402 로직 출처: servo_core/src/cia402.cpp 미러 (cia402.hpp).
// SOEM 시퀀스 출처: servo_core/src/ecat_master.cpp 주석(debt-002)의 설계 시퀀스
//         ec_init → ec_config_init → PDO매핑(0x1C12/0x1C13) → ec_config_map
//         → ec_configdc → SAFE_OP → OP.
//
// ⚠ 안전: 본 프로그램은 실제 모터를 회전시킨다. 비상정지 수단을 확보하고
//         축 주변 안전을 확인한 뒤 enable/jog 하라. Ctrl-C 시 감속·서보 OFF 시도.

#include "cia402.hpp"

#include <soem/ethercat.h> // ros-humble-soem (servo_core 주석과 동일 include 경로)

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

// --- PDO 레이아웃 (servo_core/ecat_master.hpp 의 RxPdo/TxPdo 와 동일 매핑) ---
// EtherCAT 은 little-endian. 매핑 순서 = setup_l7nh() 의 SDO 매핑 순서와 일치해야 한다.
#pragma pack(push, 1)
struct RxPdo // 마스터 → 드라이브
{
    uint16_t controlword;     // 0x6040
    int8_t mode;              // 0x6060 (Modes of operation)
    int32_t target_position;  // 0x607A
    int32_t target_velocity;  // 0x60FF
    uint32_t profile_velocity;// 0x6081
};
struct TxPdo // 드라이브 → 마스터
{
    uint16_t statusword;     // 0x6041
    int8_t mode_display;     // 0x6061
    int32_t position_actual; // 0x6064
    int32_t velocity_actual; // 0x606C
    int16_t torque_actual;   // 0x6077
    uint16_t error_code;     // 0x603F
};
#pragma pack(pop)
static_assert(sizeof(RxPdo) == 15, "RxPdo must be packed to 15 bytes");
static_assert(sizeof(TxPdo) == 15, "TxPdo must be packed to 15 bytes");

namespace
{
char g_iomap[4096];
volatile std::sig_atomic_t g_sigint = 0;

// 외부(키보드) → 주기 스레드 명령
struct Command
{
    std::atomic<bool> enable{false};   // false → shutdown(서보 OFF)
    std::atomic<bool> fault_reset{false};
    std::atomic<int> direction{0};     // +1 정회전 / -1 역회전 / 0 정지
    std::atomic<int32_t> jog_vel{0};   // 0x60FF Target Velocity (UU/s). 0 = 무회전(안전).
};
Command g_cmd;

// 주기 스레드 → 메인(출력)용 상태 스냅샷
std::mutex g_status_mutex;
TxPdo g_status_snapshot{};
cia402::DriveState g_state_snapshot = cia402::DriveState::kUnknown;
uint64_t g_cycle = 0;

void on_sigint(int) { g_sigint = 1; }

// SDO 쓰기 헬퍼 (값 타입 추론).
template <typename T> bool sdo_write(uint16_t slave, uint16_t index, uint8_t sub, T value)
{
    int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(T), &value, EC_TIMEOUTRXM);
    if (wkc <= 0)
        std::fprintf(stderr, "SDO write 실패: 0x%04X:%02X (wkc=%d)\n", index, sub, wkc);
    return wkc > 0;
}

// PreOP→SAFEOP 천이 중 호출되는 슬레이브 설정 훅: PDO 매핑을 명시적으로 구성.
// 매핑 엔트리 = (index<<16)|(subindex<<8)|bitlen.  manual: RxPDO 0x1600 / TxPDO 0x1A00,
// Sync Manager 할당 0x1C12(Rx)/0x1C13(Tx). (L7NH manual §3 PDO Mapping 참조)
int setup_l7nh(uint16_t slave)
{
    bool ok = true;
    // 1) Sync Manager PDO 할당 해제 (재구성 전 0)
    ok &= sdo_write<uint8_t>(slave, 0x1C12, 0x00, 0);
    ok &= sdo_write<uint8_t>(slave, 0x1C13, 0x00, 0);

    // 2) RxPDO(0x1600) 재매핑 — RxPdo 구조체 순서와 동일
    ok &= sdo_write<uint8_t>(slave, 0x1600, 0x00, 0);
    ok &= sdo_write<uint32_t>(slave, 0x1600, 0x01, 0x60400010); // Controlword  u16
    ok &= sdo_write<uint32_t>(slave, 0x1600, 0x02, 0x60600008); // Modes of op  i8
    ok &= sdo_write<uint32_t>(slave, 0x1600, 0x03, 0x607A0020); // Target pos   i32
    ok &= sdo_write<uint32_t>(slave, 0x1600, 0x04, 0x60FF0020); // Target vel   i32
    ok &= sdo_write<uint32_t>(slave, 0x1600, 0x05, 0x60810020); // Profile vel  u32
    ok &= sdo_write<uint8_t>(slave, 0x1600, 0x00, 5);

    // 3) TxPDO(0x1A00) 재매핑 — TxPdo 구조체 순서와 동일
    ok &= sdo_write<uint8_t>(slave, 0x1A00, 0x00, 0);
    ok &= sdo_write<uint32_t>(slave, 0x1A00, 0x01, 0x60410010); // Statusword     u16
    ok &= sdo_write<uint32_t>(slave, 0x1A00, 0x02, 0x60610008); // Modes display  i8
    ok &= sdo_write<uint32_t>(slave, 0x1A00, 0x03, 0x60640020); // Position act   i32
    ok &= sdo_write<uint32_t>(slave, 0x1A00, 0x04, 0x606C0020); // Velocity act   i32
    ok &= sdo_write<uint32_t>(slave, 0x1A00, 0x05, 0x60770010); // Torque act     i16
    ok &= sdo_write<uint32_t>(slave, 0x1A00, 0x06, 0x603F0010); // Error code     u16
    ok &= sdo_write<uint8_t>(slave, 0x1A00, 0x00, 6);

    // 4) Sync Manager 에 PDO 재할당
    ok &= sdo_write<uint16_t>(slave, 0x1C12, 0x01, 0x1600);
    ok &= sdo_write<uint8_t>(slave, 0x1C12, 0x00, 1);
    ok &= sdo_write<uint16_t>(slave, 0x1C13, 0x01, 0x1A00);
    ok &= sdo_write<uint8_t>(slave, 0x1C13, 0x00, 1);

    // 5) 기본 모드 = Profile Velocity(3), 프로파일 파라미터 보수적 기본값.
    //    UU 단위 스케일은 드라이브 설정 의존(servo_core debt-004) — 현장 교정 필요.
    ok &= sdo_write<int8_t>(slave, 0x6060, 0x00, 3);          // PV
    sdo_write<uint32_t>(slave, 0x6083, 0x00, 100000);         // Profile accel (UU/s^2)
    sdo_write<uint32_t>(slave, 0x6084, 0x00, 100000);         // Profile decel (UU/s^2)

    std::printf("[setup] slave %u PDO 매핑 %s\n", slave, ok ? "완료" : "일부 실패(로그 확인)");
    return 1; // 훅은 항상 1 반환(SOEM 규약)
}

void print_status()
{
    std::lock_guard<std::mutex> lk(g_status_mutex);
    const TxPdo &t = g_status_snapshot;
    std::printf("[status] cyc=%llu state=%s mode=%d pos=%d vel=%d torque=%d err=0x%04X %s%s\n",
                static_cast<unsigned long long>(g_cycle), cia402::state_name(g_state_snapshot),
                t.mode_display, t.position_actual, t.velocity_actual, t.torque_actual, t.error_code,
                cia402::is_target_reached(t.statusword) ? "[target_reached]" : "",
                cia402::has_fault(t.statusword) ? "[FAULT]" : "");
}

// 주기 process data 교환 스레드(약 2ms). OP 유지 + 명령 반영.
void cyclic_loop(RxPdo *rx, TxPdo *tx, std::atomic<bool> *running)
{
    using namespace std::chrono;
    const auto period = milliseconds(2);
    bool prev_reset = false;

    while (running->load())
    {
        const auto next = steady_clock::now() + period;

        // 직전 주기에 드라이브가 보낸 TxPDO 해석
        cia402::DriveState st = cia402::decode_state(tx->statusword);

        // 명령 → Controlword/Targets
        uint16_t cw;
        const bool reset_req = g_cmd.fault_reset.load();
        if (reset_req)
        {
            cw = cia402::controlword_fault_reset(); // bit7=1 (상승에지)
            if (prev_reset)                          // 한 주기 뒤 자동 해제(에지 생성)
                g_cmd.fault_reset.store(false);
        }
        else if (g_cmd.enable.load())
        {
            cw = cia402::controlword_to_enable(st);
        }
        else
        {
            cw = cia402::controlword_disable();
        }
        prev_reset = reset_req;

        rx->controlword = cw;
        rx->mode = 3; // Profile Velocity
        rx->profile_velocity = 0;
        // 회전 명령: OperationEnabled + enable + 방향 지정일 때만 속도 인가
        const bool can_move = (st == cia402::DriveState::kOperationEnabled) && g_cmd.enable.load();
        const int dir = g_cmd.direction.load();
        rx->target_velocity = (can_move && dir != 0) ? (g_cmd.jog_vel.load() * dir) : 0;
        rx->target_position = tx->position_actual; // PV 모드라 미사용, 안전상 현재값

        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        {
            std::lock_guard<std::mutex> lk(g_status_mutex);
            g_status_snapshot = *tx;
            g_state_snapshot = cia402::decode_state(tx->statusword);
            ++g_cycle;
        }

        std::this_thread::sleep_until(next);
    }
}

void print_menu()
{
    std::puts("\n==== L7NH EtherCAT 모터 테스트 ====");
    std::puts(" e: enable(서보 ON)   d: disable(서보 OFF)");
    std::puts(" f: jog 정회전(+)     r: jog 역회전(-)     x: 정지");
    std::puts(" v <n>: jog 속도 설정(0x60FF, UU/s)        c: fault reset");
    std::puts(" p: 상태 1회 출력     q: 종료(감속·서보 OFF)");
    std::puts("===================================");
}

} // namespace

int main(int argc, char **argv)
{
    const std::string ifname = (argc > 1) ? argv[1] : "eno1";
    const int32_t init_vel = (argc > 2) ? std::atoi(argv[2]) : 0;
    g_cmd.jog_vel.store(init_vel);

    std::signal(SIGINT, on_sigint);

    std::puts("⚠ 안전: 실제 모터가 회전합니다. 비상정지 수단·축 주변 안전을 확인하세요.");
    std::printf("EtherCAT 인터페이스: %s,  초기 jog 속도: %d UU/s\n", ifname.c_str(), init_vel);

    // 1) 마스터 초기화 (raw socket → root 필요)
    if (ec_init(ifname.c_str()) <= 0)
    {
        std::fprintf(stderr, "ec_init(%s) 실패 — root 권한/인터페이스명을 확인하세요.\n", ifname.c_str());
        return 1;
    }

    // 2) 슬레이브 스캔
    if (ec_config_init(FALSE) <= 0)
    {
        std::fprintf(stderr, "슬레이브를 찾지 못했습니다. EtherCAT 결선/전원을 확인하세요.\n");
        ec_close();
        return 1;
    }
    std::printf("발견된 슬레이브: %d개 (slave1=%s)\n", ec_slavecount, ec_slave[1].name);

    // 3) PDO 매핑 훅 등록(첫 슬레이브 대상)
    ec_slave[1].PO2SOconfig = setup_l7nh;

    // 4) PDO 매핑 + 분산클럭
    ec_config_map(g_iomap);
    ec_configdc();

    // 5) SAFE_OP 대기
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_slave[0].state != EC_STATE_SAFE_OP)
    {
        std::fprintf(stderr, "SAFE_OP 천이 실패 (state=0x%02X)\n", ec_slave[0].state);
        ec_close();
        return 1;
    }

    RxPdo *rx = reinterpret_cast<RxPdo *>(ec_slave[1].outputs);
    TxPdo *tx = reinterpret_cast<TxPdo *>(ec_slave[1].inputs);

    // 6) OP 천이 — process data 를 보내며 전환(워치독)
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);
    for (int i = 0; i < 200 && ec_slave[0].state != EC_STATE_OPERATIONAL; ++i)
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 5000);
    }
    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        std::fprintf(stderr, "OP 천이 실패 (state=0x%02X)\n", ec_slave[0].state);
        ec_close();
        return 1;
    }
    std::puts("OP 상태 진입 — 주기 교환 시작.");

    // 7) 주기 스레드 시작
    std::atomic<bool> running{true};
    std::thread cyclic(cyclic_loop, rx, tx, &running);

    // 8) 명령 루프
    print_menu();
    std::string line;
    while (!g_sigint)
    {
        std::printf("cmd> ");
        std::fflush(stdout);
        if (!std::getline(std::cin, line))
            break;
        std::istringstream iss(line);
        std::string c;
        iss >> c;
        if (c.empty())
            continue;

        if (c == "q")
            break;
        else if (c == "e")
        {
            g_cmd.enable.store(true);
            std::puts("enable 요청");
        }
        else if (c == "d")
        {
            g_cmd.enable.store(false);
            g_cmd.direction.store(0);
            std::puts("disable 요청");
        }
        else if (c == "f")
        {
            g_cmd.direction.store(+1);
            std::printf("jog 정회전 (vel=%d UU/s)\n", g_cmd.jog_vel.load());
        }
        else if (c == "r")
        {
            g_cmd.direction.store(-1);
            std::printf("jog 역회전 (vel=%d UU/s)\n", g_cmd.jog_vel.load());
        }
        else if (c == "x")
        {
            g_cmd.direction.store(0);
            std::puts("정지(target velocity 0)");
        }
        else if (c == "v")
        {
            int32_t v = 0;
            if (iss >> v && v >= 0)
            {
                g_cmd.jog_vel.store(v);
                std::printf("jog 속도 = %d UU/s\n", v);
            }
            else
                std::puts("사용법: v <0 이상 정수>");
        }
        else if (c == "c")
        {
            g_cmd.fault_reset.store(true);
            std::puts("fault reset 요청");
        }
        else if (c == "p")
            print_status();
        else
            std::puts("알 수 없는 명령");
    }

    // 9) 안전 종료: 정지 → 서보 OFF → 주기 정지 → OP 해제
    std::puts("\n종료 중: 감속·서보 OFF...");
    g_cmd.direction.store(0);
    g_cmd.enable.store(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 감속·shutdown 반영 시간
    running.store(false);
    if (cyclic.joinable())
        cyclic.join();

    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
    std::puts("종료 완료.");
    return 0;
}
