---
id: 2026-07-05-003
type: mistake
category: context-missing
status: closed
reflected_assets:
  - /home/amap/.claude/projects/-home-amap-Project-kkw/memory/verify-network-before-asserting.md
---

# 2026-07-05 13:33 (KST) — SSH 접속 IP를 실제 조사 없이 두 개 나열

## 무엇을 했는가
SSH 서버 설정 안내 중, `hostname -I` 가 출력한 두 주소(192.168.1.50, 192.168.1.28)를 그대로 "주 IP / 또는" 형식으로 접속 대상 후보 두 개로 제시했다. 어느 인터페이스가 유선/무선인지, 어느 경로가 실제 활성(기본 라우트)인지 조사하지 않았다.

## 무엇이 잘못이었나
`hostname -I` 의 다중 주소를 모두 동등한 접속 대상으로 가정했다. 실제로는:
- 192.168.1.50 = `enp1s0` (유선)
- 192.168.1.28 = `wlo1` (무선), **기본 라우트가 이쪽(`default via 192.168.1.1 dev wlo1`)**
사용자는 현재 **무선으로만 접속 가능**하다고 정정 → 올바른 접속 대상은 192.168.1.28 하나뿐이었다. 검증 없는 가정으로 사용자에게 왕복 정정 비용을 발생시켰다.

## 사용자 지적
> "이 PC의 계정은 amap, IP는 두 개 감지됐습니다: < - 또 조사없이   무선으로만 현재 가능."

"또"(again) — 조사 없이 단정하는 패턴의 반복 지적.

## 원인 분석
category: **context-missing** — 필요한 컨텍스트(활성 네트워크 경로·인터페이스 종류)를 조사하지 않고 작성. 명시된 "제시 전 네트워크 조사" 규칙이 설치돼 있지 않아 지식 공백으로 분류(rule-violation 아님). `ip -brief addr` / `ip route show default` 한 번이면 확인 가능했으나 첫 캡처의 `hostname -I` 출력에 안주했다.

## 재발 방지
지식 자산으로 메모리 신설: [verify-network-before-asserting](../../.claude/projects/-home-amap-Project-kkw/memory/verify-network-before-asserting.md) — IP/인터페이스/접속 안내 전 `ip -brief addr` + `ip route show default` 로 활성 경로·무선/유선 구분 후 활성 경로만 제시. MEMORY.md 인덱스에 등재.
