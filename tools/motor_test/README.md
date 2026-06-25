# L7NH EtherCAT 모터 제어 테스트 (motor_test)

LS Electric **L7NH** 서보 드라이브(**CANopen over EtherCAT, CoE**)를 EtherCAT으로 직접
제어하는 단독 테스트 도구. **enable/disable · 상태/엔코더 읽기 · Profile Velocity(jog) 회전**.

C++(SOEM)와 Python(pysoem) **두 버전**을 함께 제공한다. 동작·PDO 매핑·CiA402 로직은 동일하다.

```
tools/motor_test/
├── motor_test.cpp     # C++ 버전 (SOEM)
├── cia402.hpp         # CiA402 상태머신 (servo_core/cia402.cpp 미러)
├── CMakeLists.txt
├── motor_test.py      # Python 버전 (pysoem)
└── README.md
```

---

## ⚠ 안전 (먼저 읽을 것)

- 본 도구는 **실제 서보 모터를 회전**시킨다. 비상정지(E-Stop) 수단을 확보하고 축 주변
  간섭물·사람을 제거한 뒤 실행하라.
- `jog 속도(0x60FF Target Velocity)`는 **UU/s(User Unit/s)** 단위이며 스케일은 드라이브
  설정(전자기어비·분해능)에 의존한다. **반드시 작은 값부터** 시작해 거동을 확인하라.
  기본값은 `0`(무회전)이며, `v <n>` 로 설정하기 전엔 `f`/`r` 가 모터를 돌리지 않는다.
- 종료(`q`/Ctrl-C) 시 자동으로 감속·서보 OFF·OP 해제를 시도한다.

---

## 사전 요구

- 보드 이더넷 포트(예: `eno1`)를 L7NH EtherCAT IN 포트에 직접 연결.
- EtherCAT 마스터는 **raw socket** 을 쓰므로 **root 권한(sudo)** 으로 실행해야 한다.
- 인터페이스명 확인: `ip -br link` (이 보드는 `eno1`).

---

## A. C++ 버전 (SOEM)

### 1) SOEM 설치 (소스 빌드, sudo 불필요)
> apt(ros-humble-soem)는 이 보드(arm64)에 없어 소스 빌드로 설치한다.

```bash
cd ~
git clone --depth 1 https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
cmake -B build -DCMAKE_INSTALL_PREFIX=$HOME/.local
cmake --build build -j4
cmake --install build      # ~/.local 에 설치 (sudo 불필요)
```

### 2) 빌드
```bash
cd ~/Welding_Robot_Ros2_ws/tools/motor_test
cmake -B build
cmake --build build -j4
```
> SOEM include 경로가 다르면 CMake 출력의 `SOEM include/lib` 메시지로 확인.

### 3) 실행 (root)
```bash
sudo ./build/motor_test eno1            # jog 속도 0(무회전)으로 시작
sudo ./build/motor_test eno1 50000      # 초기 jog 속도 50000 UU/s
```

---

## B. Python 버전 (pysoem)

### 1) 설치
```bash
pip3 install --user pysoem
```

### 2) 실행 (root)
> `--user` 설치 시 sudo 환경에서 모듈을 못 찾을 수 있어 `sudo -E` + PYTHONPATH 권장.
```bash
cd ~/Welding_Robot_Ros2_ws/tools/motor_test
sudo -E env "PYTHONPATH=$HOME/.local/lib/python3.10/site-packages" \
    python3 motor_test.py eno1
# 또는 시스템 설치: sudo pip3 install pysoem  후  sudo python3 motor_test.py eno1
```

---

## 명령어 (공통)

| 키 | 동작 |
|----|------|
| `e` | enable (Shutdown→SwitchOn→EnableOperation 순차 천이) |
| `d` | disable (서보 OFF) |
| `f` / `r` | jog 정회전(+) / 역회전(−) |
| `x` | 정지 (Target Velocity 0) |
| `v <n>` | jog 속도 설정 (0x60FF, UU/s) |
| `c` | fault reset (Controlword bit7 상승에지) |
| `p` | 상태 1회 출력 (state·position·velocity·error) |
| `q` | 종료 (감속·서보 OFF·OP 해제) |

---

## 설계 노트 (출처·검증)

- **CiA402 상태머신**: `servo_core/src/cia402.cpp` 미러(동일 DS402 표준 상수). 전송 계층과
  무관한 순수 로직이라 EtherCAT/CANopen 양쪽에 동일 적용.
- **PDO 매핑**: RxPDO `0x1600`, TxPDO `0x1A00`, Sync Manager 할당 `0x1C12`/`0x1C13`.
  근거: `L7NH_200V_Manual_V1.4_EN` §3 (PDO Mapping). RxPDO/TxPDO 필드 순서는
  `servo_core/include/servo_core/ecat_master.hpp` 의 RxPdo/TxPdo 와 동일.
- **모드**: Profile Velocity = `0x6060`=3 (manual §4.4.2, page 4-16). 속도 `0x60FF`,
  프로파일 `0x6081`/가감속 `0x6083`/`0x6084`.
- **SOEM 시퀀스**: `ec_init → config_init → PDO매핑(config_func) → config_map → configdc
  → SAFE_OP → OP`. 근거: `servo_core/src/ecat_master.cpp` 의 debt-002 설계 주석.

## 알려진 한계 (debt)

- **UU 스케일 미교정** (cf. servo_core units.hpp / debt-004): `0x60FF` 속도 단위계가
  드라이브 전자기어 설정에 의존. 현장에서 `v` 값을 점진적으로 올리며 실측 교정 필요.
- **하드웨어 미검증**: 작성 시점 보드 `eno1` 미결선(NO-CARRIER) 상태라 컴파일·문법까지만
  검증됨. 실제 모터 거동은 결선 후 사용자 실행으로 확인해야 한다.
- 본 도구는 servo_core 의 미완성 EtherCAT 계층(debt-002)의 **참조 구현**으로 활용 가능.
