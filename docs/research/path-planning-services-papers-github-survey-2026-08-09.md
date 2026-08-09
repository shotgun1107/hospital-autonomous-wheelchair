# 병원 자율휠체어 실제 서비스·논문·오픈소스 보강 조사

- 조사일: 2026-08-09
- 상태: 비권위 조사 자료 — 기술 채택 아님
- 상위 기준: [경로 기능 1단계 기준선](../product/path-planning-baseline.md)
- 기능 공간: [경로 기능 2단계 추상 시나리오](../product/path-planning-functional-scenarios.md)
- 비교 입력: [경로 기능 3단계 의존성 목록](../product/path-planning-dependencies.md)
- 선행 조사: [경로·장애물 대응 폭넓은 문헌 조사](navigation-obstacle-literature-review-2026-08-09.md)
- 목적: 실제 운영 서비스, 현장 실증, 원 논문, 공개 GitHub·데이터셋과 원 분야 밖의 유사 구조를 조사해 다음 단계의 `전체 경로 재선택 / 제한적 현재 경로 수정 / 조합형` 비교 근거를 보강한다.

이 문서는 SLAM, 센서, 좌표계, 알고리즘, 프레임워크 또는 수치 임계값을 채택하지 않는다. 공개 자료에 적힌 기능을 우리 제품의 기능으로 간주하지 않으며, 시뮬레이션 성공을 사람 탑승 안전의 증거로 사용하지 않는다.

## 한눈에 보는 결론

1. 프로젝트와 가까운 서비스는 이미 국내외에 존재한다. 자율휠체어 자체나 무인 복귀 자체를 독창성으로 주장할 수 없다.
2. 본 조사에서 확인한 승객 이동 사례에는 `등록 승차점·목적지 + 사전 설정·매핑 경로 + 로봇 로컬 정지 + 하차 뒤 빈 차 복귀`가 반복해서 나타났다. 병원 전체의 임의 위치에서 부르는 택시형 서비스가 상시 운영된다는 강한 근거는 찾지 못했다.
3. 분당서울대병원의 `Wheelie`는 앱 기반 서비스 신청·이용, 무인 회수, 원격 위치·상태·배터리 확인까지 공개돼 있지만, 임의 위치로 빈 차를 부르는 hailing은 확인되지 않았다. 2022년 원 발표와 후속 병원 소개의 탑승 중 자율주행 표현도 다르므로 기능 진화 여부를 확인하기 전에는 `승객 탑승 완전자율 운행`으로 단정하면 안 된다.
4. Toyota 병원 물류로봇 `Potaro`의 실제 운영 자료는 `사람 감지 → 감속·정지 → 일정 시간 뒤 재계산` 방식이 군중에서 장시간 정지와 반복 재계산을 만들 수 있음을 보여준다. 이는 정지 이후 판단을 단순 타이머 하나로 충분하다고 가정하지 말고 후속 비교 항목으로 검증해야 한다는 현장 시사점이다.
5. 문헌은 국소 수정과 전체 재선택의 사용 조건을 구분하는 근거를 제공한다. 기존 경로 주변에 안전한 이탈·재합류 공간이 남아 있으면 국소 수정이 후보가 된다. 문·복도 폐쇄처럼 연결성이 끊겼을 때 다른 등록 통로로 자동 계속하려면 전체 재선택이 후보가 되며, 그렇지 않으면 기준선대로 정지 상태를 유지한다. 병원 승객 휠체어에서 세 방식을 동일 조건으로 비교해 조합형의 우월성을 입증한 연구는 없다.
6. 조사한 일부 실제 시스템과 성숙한 오픈소스는 `전역 경로 / 국소 움직임 / 계획 검증 / 명령 게이트 / 물리 정지`를 서로 다른 책임으로 둔다. 하나의 planner 성공률로 나머지 계층을 증명할 수 없다.
7. 현재 대회에는 대형 프레임워크 전체 도입보다 구조와 시험법을 선별해 참고하는 편이 현실적이다. Python은 결정론적 논리·회귀시험, Unity는 장면·설명·UX, ROS·Gazebo 계열은 나중의 폐루프 통합시험에 알맞다.

## 조사 방법과 증거 등급

이번 조사는 체계적 문헌고찰이 아니라 범위를 넓게 탐색하는 스코핑 조사다. 자율휠체어뿐 아니라 병원 물류 AMR, 공항 이동서비스, 창고·교통·자동차·항공 안전 구조, 게임 내비게이션, 사회적 내비게이션과 다중 로봇을 포함했다.

| 등급 | 의미 | 이 문서에서의 해석 |
|---|---|---|
| O | 실제 시설의 반복 운영·정식 도입을 시설·운영기관 자료 또는 공식 배치 발표로 확인 | 운영 사실에 대한 근거다. 제조사만 제공한 세부 기능·수치는 `M`을 함께 붙이고 비공개 내부 동작까지 추정하지 않는다. |
| P | 병원·공항 등 실제 현장 파일럿·실증 | 현장 가능성은 보여주지만 상시 운영·안전성은 증명하지 않는다. |
| R-H | 사람 또는 실물 로봇을 사용한 연구 | 해당 조건의 실험 근거이며 제품·임상 일반화는 제한된다. |
| R-S | 시뮬레이션·이론 연구 | 후보 원리와 시험 설계에 사용하며 물리 안전 근거로 쓰지 않는다. |
| OSS | 공개 코드·문서·데이터 | 구조·시험 자산 후보이며 유지보수·라이선스·버전을 별도 확인한다. |
| M | 제조사 기능·수치 주장 | 사실 후보로 기록하되 독립 검증 수치와 분리한다. |

소스 우선순위는 병원·공항·정부·연구기관 공식 자료, 원 논문, 프로젝트 공식 문서·저장소, 제조사 자료 순으로 두었다. 운영·파일럿·연구·마케팅이 섞여 있으면 더 낮은 근거 수준으로 해석했다.

복합 등급 `O/M`은 공식 자료에서 실제 운영·정식 도입은 확인되지만, 운영 근거 자체 또는 세부 기능·수치가 제조사 자료에 의존한다는 뜻이다. `O/P/M`처럼 여러 등급이 붙으면 실제 운영, 현장 개선 실증, 제조사 주장 부분이 한 사례 안에 함께 있다는 뜻이며 각 부분의 증거 강도를 합쳐서 해석하지 않는다. 의존성 표기의 `MAP-01/04`는 `MAP-01, MAP-04`처럼 같은 접두사를 생략한 축약이다.

## 1. 실제 승객·환자 이동 서비스

### 1.1 직접 사례 비교

| 사례 | 등급 | 공개 자료에서 확인된 운영 구조 | 확인되지 않았거나 주의할 점 | 주요 의존성 |
|---|---|---|---|---|
| [WHILL 하네다공항 현행 서비스](https://tokyo-haneda.com/en/service/facilities/whill.html), [운영 확대 발표](https://tokyo-haneda.com/site_resource/whats_new/pdf/000011064.pdf) | O | 정해진 승차장에서 탑승해 화면으로 등록 게이트를 선택한다. 센서 기반 정지·충돌회피, 일시정지·비상정지 버튼, 하차 뒤 빈 차 대기소 자동복귀가 공개돼 있다. | 임의 현재 위치 호출이 아니다. 경로 설정·안전 구역·이용 자격을 운영자가 미리 제한한다. | MAP-01/04/06/07/09, LOC-01/02/03/05, OBS-01/04/05/06, VEH-01/02/04/05, SAFE-01/03/04/05/07, SYS-04/05/07, VAL-04/05 |
| [요코하마시립시민병원 WHILL](https://yokohama-shiminhosp.jp/introduction/whill.html) | O | 병원 정문 승차장에서 사전 설정 목적지를 고르고 이동하며, 하차 후 정문 승차장으로 무인 복귀한다. | 병실·현재 위치로 빈 차가 직접 찾아가는 운영 근거는 아니다. 장애물 뒤 재개 권한은 비공개다. | MAP-01/04/07, LOC-01/02, SAFE-01/04, SYS-04/05, VAL-04/05 |
| [게이오대학병원 WHILL](https://www.hosp.keio.ac.jp/about/feature/aihospital/), [초기 기술실증](https://whill.inc/us/keio-university-hospital-begins-technical-trial-of-whill-autonomous-driving-technology-to-improve-patient-mobility/) | O/P | 초기 정문↔1층 접수처 실증 뒤 현재 병원 안내에 층별 운영시간과 노선이 공개돼 있다. 지정 위치 하차 후 자동복귀 구조다. | 현행 서비스 전환 시점과 내부 장애물 정책은 공개 자료만으로 특정하기 어렵다. | MAP-01/04/06/07, LOC-01/02/05, OBS-01/04/06, SAFE-01/03/04, SYS-04/05, VAL-04/05/06 |
| [분당서울대병원 Wheelie](https://www.snubh.org/dh/module/en_ihcstoryView.do?DP_CD=EN&MENU_ID=003011&NO=127&cPage=9), [2022년 원 발표](https://www.newswire.co.kr/newsRead.php?no=947638) | P/M | 앱 기반 서비스 신청·이용, SLAM·LiDAR, 5G 원격 위치·상태·배터리 확인, 빈 차의 지정 대기구역 자동복귀, 승하차 자동고정이 공개돼 있다. | 임의 위치 hailing은 확인되지 않았다. 원 발표는 탑승 중 `전동주행 경로 안내`와 무인 회수 모드를 구분하지만 후속 병원 글은 자율 환자 이송으로 표현한다. 완전자율 탑승 운영으로 단정하지 않는다. | MAP-01/04/07, LOC-02/03/05, OBS-01/02/04/06, VEH-03/04/05, SAFE-01/02/03/04/05/07, SYS-03/04/05/07, VAL-04/05/06 |
| [WHILL 마이애미공항](https://news.miami-airport.com/miami-dade-mayor-and-american-airlines-announce-new-autonomous-wheelchairs-at-mia/) | O | 10대를 운영하며 승객이 화면에서 등록 게이트를 고른다. 공항 발표 기준 하루 평균 120명이 이용한다. | 제조사·공항 수치는 운영 규모 근거이지 모든 장애 상황의 성공률 근거가 아니다. | MAP-01/04/07, LOC-01/02, OBS-01/04, VEH-05, SAFE-01/04/05, SYS-04/05, VAL-04/05 |
| [WHILL 디트로이트공항](https://whill.inc/us/news/unifi-aviation-launches-whill-autonomous-wheelchair-service-at-detroit-metro-airport) | P | 체크인 때 신청하고 전용 지점에서 옮겨 탄 뒤 사전 매핑 경로를 운행하며 빈 차가 복귀한다. | 독립적으로 옮겨 탈 수 있는 승객만 대상이다. 서비스 자격과 인계가 자율주행과 별도 요구임을 보여준다. | MAP-01/04/07, LOC-01/02, VEH-05, SAFE-01/05, VAL-05/06 |
| [SMART·MIT·NUS 자율휠체어](https://portal.smart.mit.edu/news-events/smart-fm-trials-self-driving-wheelchair) | P/R-H | Changi General Hospital에서 실제 병원 시험을 수행했다. | 호출·복귀·지도 갱신·재개 권한을 공식 자료만으로 확인할 수 없어 실제 서비스로 분류하지 않는다. | LOC/OBS 후보, VAL-03/04/06 |
| [Connected Driverless Wheelchair](https://pmc.ncbi.nlm.nih.gov/articles/PMC7898257/) | R-H | 병원 시스템 요청, 환자 ID·위치, 빈 차 픽업, 환자 운송, 하차, 대기 지점 복귀를 연구 프로토타입으로 종단 구현했다. | 코로나19로 실제 병원 통합시험을 못 했고, 장애물 대응은 정지·대기이며 우회는 후속 과제였다. | MAP-01/03/07, LOC-01/02, OBS-01/05, SAFE-01/02/04/06, SYS-01/04/05, VAL-02/04/06 |
| [2026 원격 hailing 자율 전동휠체어](https://arxiv.org/abs/2607.06383) | R-H 초기 POC | 상용 self-balancing 휠체어에 인지·내비게이션을 붙여 멀리서 부르는 hailing과 사람 추종을 실물로 시연했다. `빈 차가 찾아오는 기능`의 제한적 연구 근거다. | 무인 상태, 제한된 실내, 모터 토크 제한, 부분 센서 범위의 초기 시연이다. 탑승자·병원·혼잡 환경 운영 근거가 아니다. | LOC-02/05, OBS-03/04/06, VEH-02/04/05, SAFE-03/07, VAL-03/04/06 |
| [Tsukuba 병원 WHILL 환자 연구](https://pmc.ncbi.nlm.nih.gov/articles/PMC11067163/) | R-H 임상 예비연구 | 실제 환자 51명이 사전 지도화한 약 100m 제한 경로를 이용했고 충돌은 없었다. | 26명은 위험하게 느낀 순간을 보고했으며 보행자 근처 통과와 갑작스러운 정지가 주요 원인이었다. `충돌 0`과 체감 안전을 분리해야 한다. | OBS-03/04, VEH-03/04/05, SAFE-03/07, VAL-04/05/06 |

### 1.2 승객 서비스에서 반복되는 운영 패턴

- 출발점과 목적지는 미리 등록되고 운영자가 허용 경로를 관리한다.
- 승객 운행과 빈 차 복귀는 같은 차체를 쓰더라도 서로 다른 미션이다.
- 승하차 가능 여부, 옮겨 타기, 보조 인력과 이용 자격은 경로 계산 밖의 별도 안전 조건이다.
- 조사 사례에서는 차량 로컬 정지와 운영자 관제가 함께 나타났다. 배차·가용성·배터리·지도 변경·예외 복구는 관제 책임 후보로 관찰됐고, 급박한 충돌 방지는 차량 가까이에 두는 사례가 반복됐다. 우리 시스템의 최종 책임 배치는 아직 확정하지 않는다.
- 공개 자료는 `자동정지`를 자주 설명하지만 `정지 완료 확인`, `누가 재출발을 승인하는가`, `오래된 경로를 언제 폐기하는가`는 거의 공개하지 않는다. SAFE-02/04/05와 SYS-07은 경쟁 제품을 보고 추정하지 말고 자체 요구로 검증해야 한다.

## 2. 병원 물류·시설 로봇에서 얻는 운영 근거

화물 AMR은 승객 휠체어와 위험 수준이 같지 않다. 다만 수년간 병원 안에서 운영되며 문, 엘리베이터, 좁은 복도, 교차로, 지도 변경, 원격 복구를 다뤘다는 점에서 경로·운영 의존성의 강한 인접 근거다.

| 사례 | 등급 | 공개된 현장 결과와 동작 | 프로젝트에 주는 교훈 | 주요 의존성 |
|---|---|---|---|---|
| [Toyota Potaro](https://global.toyota/en/mobility/frontier-research/43981344.html), [기술 리뷰](https://global.toyota/pages/global_toyota/mobility/technology/toyota-technical-review/TTR_Vol70-2_E.pdf) | O/P/M | Toyota Memorial Hospital에서 2023년부터 24대 운영, 2026년 1월 기준 성공률 99%, 누적 27,000km를 제조사가 공개했다. 기존 `감속·정지→일정시간 뒤 재계산`은 군중에서 장시간 정지·반복 재계산을 만들었다. 사람 분류·속도 예측 뒤 횡단자에게 양보하고 마주 오는 사람은 더 일찍 피하도록 개선했지만 측·후방 대응은 연구 중이다. | 단순 타이머 재계획의 충분성을 후속 비교에서 검증해야 한다. 사람의 움직임, 관측 범위, 대기 가능 장소, 대체 통로, 반복 재계획 억제를 함께 시험할 후보가 된다. 실험실 개선 뒤에도 현장에 새 실패가 남는다. | MAP-03/04/05/06/09, LOC-02/03/04/05, OBS-01~06, VEH-01~04, SAFE-01~07, SYS-01/04/05/06/07, VAL-02~06 |
| [Toyota 천장카메라 연계](https://global.toyota/en/mobility/frontier-research/40390293.html), [지도·배치 자동화](https://global.toyota/en/mobility/frontier-research/43525400.html) | O/P/M | 200대 이상 천장카메라로 대형 침대·카트의 좁은 복도 진입을 예측해 로봇을 상류에서 대기시킨다고 제조사가 공개했다. 건축도면에서 지도와 대기 지점을 만드는 현장 개선연구도 공개했다. | 모든 문제를 휠체어 탑재 센서 하나가 해결할 필요는 없다. 다만 대회 MVP가 시설 인프라를 전제로 할지는 별도 범위 결정이다. | MAP-03/05/06/09, OBS-02/03/06, SAFE-03/07, SYS-04/05/07, VAL-02/03 |
| [Kawasaki FORRO 병원 정식 도입](https://www.khi.co.jp/news/detail/20240710_1.html), [제품 공식 안내](https://forro-service.com/) | O/M | 실증 뒤 2024년 정식 도입, 3대 24시간 운영과 8,500건 이상 운송이 공개돼 있다. 제조사 안내는 우회 공간이 있으면 우회하고 없으면 정지·대기하며, 엘리베이터·보안문·자동충전·24시간 지원을 연동한다고 설명한다. | `우회 가능 / 기다림 / 지원`을 모두 정상 결과 후보로 비교해야 한다. 경로 없음은 곧 충돌 실패가 아니다. | MAP-01/03/04/06/09, LOC-01/02/05, OBS-01/02/03/05/06, SAFE-01~07, SYS-01/03/04/05/07, VAL-04/05/06 |
| [Panasonic HOSPI × Changi General Hospital](https://ap.connect.panasonic.com/sg/en/case-studies/panasonic-autonomous-delivery-robots-hospi-aid-hospital-operations-changi-general) | O/M | 2015년 시험 뒤 4대가 24시간 약품·검체·문서를 운송한다고 공개했다. 지도 기반 이동, 위치·이력 관제, 자동문·엘리베이터·충전과 다양한 이동약자·장비의 감지·감속·정지·회피를 설명한다. | 실제 병원 물체 종류는 사람/벽 두 종류가 아니다. 장애물 분류가 행동에 필요한지 여부를 후속 비교에서 따져야 한다. 제조사의 사고 없음 수치는 별도 M 근거다. | MAP-01/03/04/06, LOC-02/05, OBS-01/02/04/06, VEH-01/04, SAFE-01/03/05/06/07, SYS-03/04/05/07, VAL-04/05/06 |
| [Aethon TUG × Franciscan Health](https://www.franciscanhealth.org/about/news-and-media/franciscan-health-crown-point-robots-assisting-in-kitchen-environmental-deliveries), [제조사 관제 설명](https://aethon.com/aethon-tug-autonomous-mobile-robot-discussed-ria-magazine/) | O/M | 병원 자료는 6대의 린넨·식사 운송과 직원 전용 엘리베이터 사용을 확인한다. 제조사 자료는 막힘을 원격지원센터에서 해결하거나 현장 직원을 보내는 예외 복구 구조를 설명한다. | 원격·현장 지원은 운영 구조 후보지만, 우리 관제의 지도 변경권이나 복구 책임을 확정하는 근거는 아니다. | MAP-01/03/04/05/06, LOC-02/05, OBS-01/02/03/06, SAFE-01/02/03/04/06/07, SYS-03/04/05/07, VAL-04/05 |
| [Moxi × Mary Washington Hospital](https://www.marywashingtonhealthcare.com/news/2021/december/mary-washington-hospital-welcomes-moxi-the-robot/), [2022 결산](https://www.marywashingtonhealthcare.com/news/2022/december/mwhcs-top-10-in-2022/), [제조사 FAQ](https://www.diligentrobots.com/faq) | O/M | 병원은 4대와 17,043건의 배송 실적을 공개했다. 제조사 FAQ는 장애물을 우회하거나 없어질 때까지 기다리고 자동충전·문·엘리베이터를 연동한다고 설명한다. | 목적지 도달 성공률 외에 작업 수, 개입률, 복구시간과 운영 부담을 기록해야 한다. | MAP-01/03/04, LOC-01/02/05, OBS-01/02/05, SAFE-01/04/06, SYS-04/05, VAL-04/05 |
| [Tartu 병원 Open-RMF 현장 연구](https://pmc.ncbi.nlm.nih.gov/articles/PMC9445435/) | R-H 실제 병원 | 전역 fleet·문 조정과 개별 ROS 로컬 내비게이션을 결합해 ICU–검사실 혈액 운송을 시험했다. 지도 왜곡, 공용 Wi-Fi 음영, 혼잡 감지·안전 대기 부재, 직원용 긴급 정지·이동 필요가 드러났다. | 전역 운영과 로컬 이동을 나눠도 지도·통신·대기·복구 경계가 없으면 종단 운행이 깨진다. | MAP-03/05/06/09, LOC-04, OBS-03/06, SAFE-01/05/06, SYS-01/03/04/05/07, VAL-06 |
| [CGH RoMi-H 상호운용](https://www.cgh.com.sg/about-cgh/news/caring/issue4-2022/driving-automation-in-and-beyond-healthcare) | P | 이기종 로봇과 문·엘리베이터, 좁은 통로 충돌방지, 공통 관제를 실증했다. 생명구호 약품 운반에 더 높은 통행 우선권을 주는 운영 정책도 공개돼 있다. | 다른 로봇과의 우선권은 로컬 장애물 회피만으로 해결되지 않는다. 현재 1대 MVP에는 제외 가능하지만 D-08과 향후 다중화의 근거다. | MAP-03/09, LOC-02/05, OBS-02/03, SAFE-06, SYS-04/05/07, VAL-02/03/04/06 |
| [KTPH 수동휠체어 위치관리](https://www.a-star.edu.sg/News/astarNews/news/press-releases/a-star-collaborates-with-local-start-up-i.o.t.workz) | O 인접 서비스 | 150대의 수동휠체어가 공용구역·주차존·로비·외래 중 어디에 있는지 추적한다. | 정밀 연속좌표 관제와 별개로, 호출 전 `어느 대기구역에 사용 가능한 의자가 있는가`라는 의미적 가용성 정보는 실제 운영 가치가 있다. | LOC-05, SYS-04/05/07 |

## 3. 원 논문에서 확인한 전역·국소·조합의 경계

### 3.1 가장 직접적인 경계 근거

1993년 [Elastic Bands](https://9p.io/who/seanq/icra93.pdf)는 전역 경로를 센서 정보로 국소 변형하는 구조를 제안하면서, 작은 환경 변화에는 경로 전체를 다시 찾지 않아도 되지만 `문이 닫혀 다른 문으로 가야 하는 경우`에는 local band가 실패해 global search가 필요하다고 직접 적었다. 현대 병원이나 승객 안전 실험은 아니지만 다음 경계를 가장 명확하게 보여준다.

```text
기존 경로 주변에 허용된 이탈·재합류 공간이 남음
    → 제한적 현재 경로 수정 후보

현재 통로·문이 막혀 경로의 연결 관계가 깨짐
    → 전체 경로 재선택 후보

관측·위치·정지 상태가 불확실하거나 제동 여유가 부족함
    → 둘 중 어느 것도 실행하지 않고 정지 유지
```

[Global Dynamic Window Approach](https://cs.stanford.edu/groups/manips/publications/pdfs/Brock_1999_ICRA.pdf)는 전역 free-space 연결성과 국소 동적 회피를 결합해 반응형 local planner의 local minimum을 보완했다. 실물 로봇 데모는 있지만 소규모 정성 실험, holonomic 플랫폼이며 환자 탑승 근거는 아니다. 따라서 `조합이라는 구조가 성립한다`는 근거이지 우리 프로젝트에서 조합형이 자동으로 최선이라는 증거는 아니다.

### 3.2 후보별 대표 연구와 한계

| 자료 | 검증 수준 | 확인되는 가치 | 그대로 채택할 수 없는 이유 | 후보·의존성 |
|---|---|---|---|---|
| [D*](https://publications.ri.cmu.edu/the-d-algorithm-for-real-time-planning-of-optimal-traverses), [D* Lite](https://publications.ri.cmu.edu/d-lite) | R-S, 대표 증분 탐색 | 이동 중 일부 graph cost가 바뀔 때 이전 탐색 정보를 재사용해 경로를 갱신하는 원리 | 알 수 없는 지형 연구가 중심이고 차체·사람·정지·명령 허가는 다루지 않는다. | 전체/조합; MAP-03/05/09, SYS-06/07, VAL-02 |
| [Dynamic Window Approach](https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance) | R-H | 로봇 동역학과 속도 공간을 고려한 빠른 국소 반응을 실물·동적 환경에서 보였다. | 전역 연결성·교착 탈출을 보장하지 않으며 특정 구동 연구다. 휠체어 승객 안전 근거가 아니다. | 국소/조합; OBS-01/02, VEH-02/03/04, SAFE-03/07, SYS-06 |
| [Velocity Obstacles](https://journals.sagepub.com/doi/pdf/10.1177/027836499801700706) | R-S | 동적 장애물을 현재 점유만이 아니라 위치·속도·시간 관계로 다뤄야 함을 정식화한다. | 정확한 상태·속도와 단순한 운동 예측을 가정하며 사람 의도와 센서 지연을 해결하지 않는다. | 국소/조합; OBS-03/06, VEH-03/04, SAFE-07, SYS-07 |
| [ORCA](https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf), [RVO](https://www.cs.unc.edu/~geom/RVO/icra2008.pdf) | R-S + 소수 실물 | 많은 이동체의 상호 회피를 계산하는 효율적인 기준선이다. | 상대도 회피 책임을 나누고 유사한 규칙을 따른다는 가정이 강하다. 외부 보행자·병상·로봇이 협조한다고 가정할 수 없다. | 국소/조합·군중 생성; OBS-03/04/06, SAFE-03/07, VEH-02, SYS-07 |
| [Timed Elastic Band](https://rst.etit.tu-dortmund.de/storages/rst-etit/r/Global/Paper/Roesmann/2013_Roesmann_ECMR.PDF) | R-H/R-S | 초기 전역 경로를 시간·거리·장애물·동역학 조건으로 국소 최적화하는 대표 구조다. | 초기 topology와 가중치에 의존하며 local minimum·튜닝 문제가 있고 임상 근거가 없다. | 국소/조합; MAP-08, OBS-02/03, VEH-01/02/04, SYS-06 |
| [Dense Human Crowds](https://journals.sagepub.com/doi/10.1177/0278364914557874) | R-H, 실제 혼잡 군중 488회 | 정적 clearance나 비협조적 반응형 local만으로는 밀집 환경에서 freezing·unsafe 행동이 늘 수 있음을 보였다. | 연구의 사람 밀도·모델·수치를 병원 요구값으로 전용할 수 없다. | 조합/사회예측; OBS-03/04/06, MAP-09, SAFE-07, VAL-03/05/06 |
| [Arena-Bench](https://arxiv.org/abs/2206.05728) | R-S + TurtleBot3 실물 | 같은 전역 A* 아래 여러 local planner와 3종 운동학을 비교해 planner 순위가 로봇·환경에 따라 바뀌고 sim-real 차이가 큼을 보였다. | 이 연구의 success는 일부 조건에서 충돌을 허용해 우리 안전 합격 기준으로 사용할 수 없다. | 세 후보의 local 계층; OBS-03/04/06, VEH-02/04, SAFE-07, SYS-06/07, VAL-03~06 |
| [DynaBARN](https://people.cs.gmu.edu/~xiao/papers/dynabarn.pdf) | R-S | 장애물 속도·가감속·smoothness에 따라 DWA/TEB 등의 순위가 바뀌어 단일 보행자 모델로 planner를 고르면 안 됨을 보여준다. | 원통 장애물, 비상호작용, 시뮬레이션이며 실제 승객 운행 근거가 아니다. | 국소/조합; OBS-03/06, SAFE-07, SYS-06/07, VAL-02/03/05/06 |
| [지능형 휠체어 임상 검증](https://pmc.ncbi.nlm.nih.gov/articles/PMC3691756/) | R-H, 17명·32세션 | doorway·corridor·parking의 local 보조가 충돌을 크게 줄일 수 있음을 보였다. | point-to-point global mode는 구현됐지만 임상 검증되지 않았고, 저자도 큰 공간에는 local representation이 부족하다고 적었다. | 국소의 제한 근거; MAP-04/07/08, OBS-01/02/05, VEH-01/02, VAL-04/05/06 |
| [Wheelchair Navigation System](https://pmc.ncbi.nlm.nih.gov/articles/PMC5134465/) | R-H | local 안전 정지가 상위 vision 이동 명령보다 우선하는 계층을 실물 휠체어로 보여준다. | 일부 사용자 시험은 자율 탑승이 아니라 안내를 따른 joystick 운전이며 처리 지연과 인식 오류가 남는다. | 국소 안전 경계; OBS-04/06, SAFE-01/04/07, SYS-04/07, VEH-03/04, VAL-04~06 |
| [Inevitable Collision States](https://doi.org/10.1163/1568553042674662), [Safe Motion Planning](https://doi.org/10.1109/IROS.2005.1545549) | R-S | 동적 환경에서 부분 motion의 끝도 계속 안전하게 멈출 수 있는 상태여야 한다는 안전 껍질을 제시한다. | 보수적 근사이고 사람 예측 오류·센서 불확실성을 저절로 해결하지 않는다. | 세 후보 공통; VEH-03/04, OBS-03/04, SAFE-03/07, SYS-07, VAL-01/02 |
| [센서 시간·공간 공동 보정](https://furgalep.github.io/bib/furgale_iros13.pdf) | R-H/R-S | 센서 융합은 좌표 정합뿐 아니라 시간 offset도 문제임을 실물 자료로 보였다. | calibration 연구이며 runtime 경로 결과의 freshness나 안전 임계값을 제공하지 않는다. | 공통 입력; LOC-05, OBS-06, SYS-07, VAL-04/06 |

### 3.3 병원 배치 연구가 추가하는 요구

[2026 자율휠체어 병원 배치 요구 비전 논문](https://www.weisongshi.org/papers/guo26-SWee.pdf)은 실증 비교 논문이 아니라 vision paper이므로 요구 후보로만 사용한다. 이 논문은 실제 배치의 실패가 단순 경로 성능보다 문·엘리베이터·출입통제, 복구, 의미적 마지막 위치, 장기 지도 유지, caregiver 개입, end-to-end 지연과 업무 권한에서 생긴다고 정리한다. 현재 프로젝트의 알려진 지도·등록 지점·통제 공간 기준선과 잘 맞지만, 여기 적힌 장치·모델을 채택하는 근거는 아니다.

2024년 [병원 모바일 로봇 위험 분류 연구](https://mediatum.ub.tum.de/doc/1770691/document.pdf)는 환경 복잡성, 위생, 사람·물체 상호작용, 업무 흐름 유연성, 자율성 등 여러 위험 축을 분리한다. 경로 성공률 하나가 병원 서비스 성공을 대표할 수 없다는 보강 근거다.

## 4. 공개 GitHub·프레임워크·데이터셋 평가

### 4.1 먼저 구분해야 하는 네 책임

```text
graph/global reroute
    ≠ bounded local correction
    ≠ software command safety gate
    ≠ physical motor cutoff / E-stop
```

오픈소스 이름 하나에 네 책임이 함께 보이더라도 같은 안전 수준을 뜻하지 않는다. 특히 CPU에서 실행되는 소프트웨어 stop node를 인증된 안전 스캐너·컨트롤러·물리 비상정지와 동일하게 표현하면 안 된다.

### 4.2 유지보수되는 큰 프로젝트

| 프로젝트 | 2026-08 조사 상태 | 참고 가치 | 대회 POC에서의 판단 | 라이선스·주의 |
|---|---|---|---|---|
| [ROS 2 Nav2](https://github.com/ros-navigation/navigation2), [공식 문서](https://docs.nav2.org/) | 활발한 유지보수, 2026 릴리스 확인 | Smac 계열 전역 계획, DWB/RPP/MPPI 계열 local, behavior tree, route graph, collision monitor가 분리돼 있어 후보의 책임 경계를 연구하기 좋다. [Route graph recovery](https://docs.nav2.org/behavior_trees/trees/navigate_on_route_graph_w_recovery.html)는 사전 정의 graph와 first/last-mile freespace를 결합하고 path invalid 시 재계산하는 구조를 보여준다. | 전체 스택 채택 결정은 아직 이르다. 기본 recovery의 spin·back-up은 `먼저 정지하고 안전할 때만 이동` 원칙과 충돌할 수 있어 그대로 계승하면 안 된다. | 저장소 전체가 단일 라이선스가 아니라 package별 Apache-2.0/BSD-3/LGPL 등이 섞여 있어 실제 사용 package별 확인 필요. Nav2 Route의 지원 ROS 배포판도 확인해야 한다. |
| [Open-RMF](https://github.com/open-rmf/rmf), [RMF demos](https://github.com/open-rmf/rmf_demos) | Apache-2.0, 최근 배포 지속 | 시설 graph, lane, 문·엘리베이터, task dispatch, 경로 폐쇄, 교차로·다중 로봇 충돌과 관제를 연구할 수 있다. Clinic world도 있다. | 한 대의 축소형 POC에는 전체 도입이 과도하다. 지도·책임 분리·폐쇄 시나리오와 시각화만 참고할 가치가 높다. local collision safety를 대체하지 않는다. | ROS 2·다수 package·시설 통합 비용이 큼. |
| [Autoware Universe](https://github.com/autowarefoundation/autoware_universe) | Apache-2.0, 2026 활동·릴리스 확인 | 자동차용 전체 stack보다 `계획 결과 검증`, `명령 source 선택`, `heartbeat/timeout`, `emergency command`, `오래된 trajectory 검사` 패턴이 유용하다. | 자동차용 Lanelet·차량 모델·센서 stack은 직접 도입에 과도하다. SAFE·SYS 시험명세를 만드는 참조 아키텍처로 한정한다. | 자동차 ODD의 수치·행동을 휠체어로 복사하지 않는다. |
| [Gazebo Sim](https://github.com/gazebosim/gz-sim) | Apache-2.0, Harmonic LTS 지원 | 센서, footprint, 명령 지연, 정지 완료, 재개 조건을 포함한 ROS 폐루프 시험에 적합하다. | Python·Unity 다음의 통합 검증 후보이며 현재 구현 대상은 아니다. | Gazebo Classic은 2025년 EOL이다. 오래된 벤치 world를 그대로 기반으로 삼지 않는다. |

### 4.3 Python 배열·논리 검증에 유용한 자산

| 자산 | 사용할 수 있는 부분 | 사용하지 말아야 할 주장 | 라이선스·유지보수 주의 |
|---|---|---|---|
| [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) | A*, Dijkstra, D*, D* Lite, DWA 등 작은 예제의 동작·의사코드·시각화 비교 | 예제 코드가 우리 차체·안전·동적 사람 요구를 충족한다는 주장 | MIT. 교육·연구 예제이며 제품 stack이 아님. |
| [MovingAI grid benchmarks](https://www.movingai.com/benchmarks/) | 재현 가능한 grid와 start-goal scenario로 경로 존재성, 길이, 계산량, 폐쇄 뒤 우회 논리를 반복 시험 | 병원 공간·차체 운동·사람 안전을 대표한다는 주장 | 사이트에서 데이터 재배포 라이선스가 명확히 보이지 않아 repo에 포함하기 전 확인 필요. |
| [BARN](https://www.cs.utexas.edu/~xiao/BARN/BARN.html) | 300개 정적 밀집 환경의 통로폭·함정·footprint 난이도 아이디어 | 원본 stack의 결과를 현재 ROS·실물에 그대로 일반화 | 원본이 ROS Melodic/Ubuntu 18.04/Gazebo Classic 계열이고 데이터 라이선스 확인 필요. |
| [SocNavGym](https://github.com/gnns4hri/SocNavGym) | collision, TTC, 최소 사람 거리, personal space, stall, jerk 같은 metric·시나리오 구조 | 학습 agent 성능을 환자 안전으로 표현 | GPL-3.0과 특정 학습 stack 의존. metric 아이디어 참고가 우선. |
| [bench-mr](https://github.com/robot-motion/bench-mr) | corridor radius와 여러 wheeled planner의 통계 비교 방식 | 경량 POC 기반으로 바로 적합하다는 주장 | MIT이나 OMPL·SBPL·Boost·OpenGL 등 의존성이 무거워 연구 비교용. |

Python 1차 환경에는 대형 simulator가 필수는 아니다. 현재 승인된 26개 기능 시나리오를 작은 자체 fixture로 표현하고, 외부 benchmark는 map·상황 다양성을 보강하는 자료로만 쓰는 편이 독립성과 재현성이 높다.

### 4.4 동적 사람·사회적 내비게이션 자산

| 자산 | 장점 | 한계·권장 용도 |
|---|---|---|
| [HuNavSim](https://github.com/robotics-upo/hunav_sim) | ROS 2, 여러 사람 행동·group·social-force와 평가 metric을 제공한다. | 사람 교차·정지·머뭇거림·군집의 scenario generator 후보. 모의 행동은 임상 현실의 증거가 아니다. |
| [Arena-Rosnav](https://github.com/Arena-Rosnav/arena-rosnav) | Flatland/Gazebo/Unity/Isaac, 병원 world, 동적 사람, 여러 planner와 평가 pipeline을 제공한다. | hospital world·scenario·evaluation 구조가 유용하다. DRL training 전체를 들이면 대회 POC에 과도하고 subrepo·asset 라이선스를 따로 확인해야 한다. |
| [SocNavBench](https://github.com/CMU-TBD/SocNavBench) | 실제 보행 데이터에 grounded된 map·episode·metric을 제공한다. | MIT이지만 2021년 연구코드·작은 commit history라 유지보수형 기반보다 시나리오 참고 자료다. |
| [RVO2/ORCA](https://github.com/snape/RVO2) | 많은 agent의 반복 가능한 reciprocal avoidance 기준선을 만들 수 있다. | Apache-2.0. 보행자도 회피 책임을 나눈다는 가정 때문에 실제 안전제어 채택 근거가 아니라 군중 생성·비교 기준선이다. |
| [SUMO](https://github.com/eclipse-sumo/sumo) | 보행자를 포함한 대규모 microscopic traffic과 시간별 통로 폐쇄·rerouting을 만들 수 있다. | EPL-2.0, 활발하지만 병원 복도 수요·흐름의 논리 시험용이다. 센서, 휠체어 footprint, motor stop은 검증하지 못한다. |
| [JRDB](https://jrdb.erc.monash.edu/dataset/) | 360° RGB·LiDAR 혼잡 장면의 사람 검출·추적·예측 연구에 쓸 수 있다. | CC BY-NC-SA 3.0 제약이 있고 경로·제동 dataset이 아니다. CV 담당이 실제로 필요하다고 결정하기 전 포함하지 않는다. |

### 4.5 Unity 검증 자산

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)와 [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)는 ROS 2/Nav2 예제, C# message 연동과 시각화에 유용하다.
- 코드는 Apache-2.0이지만 Unity 엔진·에셋은 별도 라이선스를 확인해야 한다.
- Robotics Hub의 최신 정식 릴리스는 오래됐으므로 현재 Unity·ROS 버전 호환성을 사전에 확인해야 한다.
- Unity는 병원 장면, 보행자 timeline, 관제·승객에게 보이는 행동, 설명 가능한 데모를 검토하는 도구다. 실제 제동거리, 센서 지연, 물리 비상정지 또는 환자 안전을 증명하지 않는다.

[Unity NavMesh Obstacle](https://docs.unity3d.com/kr/2023.1/Manual/class-NavMeshObstacle.html)은 원 분야 밖에서 유용한 개념 분리를 보여준다. 계속 움직이는 물체는 짧은 범위의 avoidance 대상으로 다루고, 정지한 물체는 일정 시간 뒤 navmesh를 carve해 pathfinder가 다른 길을 찾게 한다. 이것은 `정적/동적이라는 label만으로 행동이 정해지는 것이 아니라, 얼마나 지속됐고 경로 연결성을 바꿨는지`를 시험하는 좋은 시각적 비유다. Unity의 동작을 실물 알고리즘으로 채택한다는 뜻은 아니다.

### 4.6 직접 의존 후보에서 우선 제외할 공개 코드

| 자산 | 이유 |
|---|---|
| [DynaBARN repo](https://github.com/aninair1905/DynaBARN) | Python 2·Gazebo Classic, 작은 commit history, 명확한 LICENSE 파일을 확인하지 못했다. 논문·world 생성 아이디어만 참고한다. |
| [nav2_social_mpc_controller](https://github.com/robotics-upo/nav2_social_mpc_controller) | 저장소가 ongoing work를 명시하고 LICENSE를 확인하기 어렵다. 직접 통합 후보에서 제외한다. |
| [TEB ROS 1 upstream](https://github.com/rst-tu-dortmund/teb_local_planner) | BSD-3이지만 ROS 1 중심이고 ROS 2 upstream 지원·릴리스 상태가 대회 stack과 맞지 않는다. 논문 비교용이다. |
| [MPC local planner](https://github.com/rst-tu-dortmund/mpc_local_planner) | GPL-3.0, ROS 1, 무거운 최적화 의존성과 낮은 최근 활동 때문에 경량 POC에 부적합하다. |
| [OpenNav smart-wheelchair perception](https://github.com/EasyWalk-PRIN/OpenNav) | MIT 연구코드로 장애물 인식 후보를 볼 수 있지만 주행·정지·재개 안전을 검증하지 않는다. |
| 학생 smart-wheelchair 저장소 | 구조·BOM 참고는 가능하지만 성숙한 유지보수형 safety stack으로 보지 않는다. 공개 라이선스·시험·하드웨어 차이를 독립 확인해야 한다. |

## 5. 원 분야 밖에서 가져온 구조적 교훈

### 5.1 자동차: 계획 생성과 실행 허가를 분리

[Autoware planning design](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/components/planning/)은 mission route, behavior, motion, validation을 분리한다. 특히 validation을 planner 밖에 두어 planner가 바뀌어도 기준선 검사를 유지한다. [Planning Validator](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator/)는 control에 넘기기 전 trajectory age, 형상과 충돌 조건 등을 검사하고 잘못된 결과를 처리한다. [vehicle command gate](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_vehicle_cmd_gate/)는 planning, external, emergency 등 여러 명령 source 중 실제 차량에 전달할 명령을 선택하고 heartbeat·timeout·속도/가감속 제한을 둔다.

우리 문서에 주는 의미는 특정 자동차 코드를 쓰자는 것이 아니다.

- 경로 기능의 `재개 가능` 결과와 실제 모터 이동 허가는 다르다.
- 계산된 경로의 생성 시점과 현재 미션·지도·장애물 상태가 맞는지 검사해야 한다.
- local safety gate가 경로·서버 명령을 거부할 수 있어야 한다.
- planner를 비교할 때 같은 외부 validation 기준을 적용해야 한다.

연결: SAFE-02/04/05/07, SYS-04/05/07, VAL-05/06.

### 5.2 항공·산업 제어: 복잡한 기능과 안전 fallback을 분리

[CMU SEI Simplex architecture](https://insights.sei.cmu.edu/library/an-architectural-description-of-the-simplex-architecture/)와 [NASA Runtime Assurance](https://ntrs.nasa.gov/citations/20240006522)는 복잡하고 완전히 검증하기 어려운 controller와 더 단순하고 신뢰할 수 있는 safety/reversion controller를 분리하는 패턴을 다룬다. 이 패턴은 `경로 알고리즘이 안전을 스스로 승인한다`는 구조를 피하는 근거다.

다만 축소형 대회 작품이 항공 수준 인증을 받는다는 뜻은 아니다. 프로젝트에는 `복잡한 재계획 결과가 이상하거나 오래됐을 때 단순한 정지 상태로 되돌아갈 수 있다`는 설계 원칙과 독립 시험 항목만 가져온다.

연결: SAFE-01/02/04/05, SYS-04/07, VAL-05/06.

### 5.3 교통: 폐쇄된 구간, 기다림과 우회

[SUMO Rerouter](https://sumo.dlr.de/docs/Simulation/Rerouter.html)는 도로 구간 폐쇄가 발생했을 때 다른 경로를 선택하거나, 우회보다 폐쇄 해제를 기다리는 편이 나으면 기존 경로에서 대기하는 논리를 제공한다. 또한 서로 다른 rerouter가 폐쇄 정보를 일관되게 보지 못하면 차량이 폐쇄 구간 사이에서 무한 반복할 수 있다고 경고한다.

병원 복도에 그대로 적용할 알고리즘은 아니지만 다음 시험을 도출한다.

- 장애물이 곧 사라질 가능성과 긴 우회 비용을 함께 고려하는가?
- 모든 planner가 같은 통행 불가 구간을 보고 있는가?
- 반복해서 두 경로를 오가는 route churn을 감지하는가?
- 대체 경로가 없을 때 잘못된 구간으로 진행하지 않고 정지하는가?

연결: MAP-05/09, OBS-03/05, SAFE-06, SYS-07, VAL-02/05.

### 5.4 창고·시설 다중 로봇: 충돌이 없어도 교착할 수 있음

[Open-RMF core](https://osrf.github.io/ros2multirobotbook/rmf-core.html)는 미래 itinerary를 공유하고 conflict를 예방하거나 negotiation으로 푸는 시설 계층을 둔다. [Kiva](https://ojs.aaai.org/aimagazine/index.php/aimagazine/article/view/2082/0)와 [Conflict-Based Search](https://doi.org/10.1016/j.artint.2014.11.006)는 알려진 통제 공간의 다중 agent 충돌을 전역 시간·경로 문제로 다룬다.

현재 MVP는 휠체어 한 대이므로 다중 fleet optimizer를 만들 이유는 없다. 그러나 외부 휠체어·병상·카트와 좁은 통로에서 서로 멈춘 C-05/D-08은 `충돌 0 = 성공`이 아님을 보여준다. 교착 지속시간, 안전 대기 위치, 사람 지원 전환을 별도 결과로 기록한다.

연결: MAP-03/09, OBS-03, SAFE-06, SYS-04/07, VAL-02/03/05.

### 5.5 게임 AI: 움직이는 물체와 경로 topology 변경을 분리

Unity NavMesh의 `moving obstacle avoidance`와 `stationary obstacle carving` 분리는 구현 채택이 아니라 시나리오 분류에 유용하다. 대상이 잠깐 멈췄다고 바로 영구 폐쇄로 바꾸거나, 지속 차단을 계속 local avoidance로만 처리하면 불필요한 경로 흔들림 또는 막힘이 생긴다.

후속 비교에서 `대상의 label`보다 아래 값을 본다.

- 현재 경로가 일부만 변형 가능한가, 연결 자체가 끊겼는가?
- 대상 상태가 얼마나 지속됐으며 판단 신뢰도는 어떤가?
- local 이탈·재합류 영역이 지도에 허용돼 있는가?
- 경로 결과가 적용될 때도 대상·지도 상태가 여전히 같은가?

연결: MAP-03/05/08, OBS-02/03/04/05, SYS-07.

## 6. 현재 세 후보에 주는 조사 결과

이 표는 우승 후보를 정하는 표가 아니다. 다음 단계의 비교에서 반드시 확인할 질문을 정리한다.

| 후보 | 문헌·운영 근거가 지지하는 사용 조건 | 추가로 요구되는 핵심 의존성 | 대표 실패 위험 |
|---|---|---|---|
| 전체 경로 재선택 | 현재 통로·문이 지속적으로 막혀 연결 관계가 바뀌고 다른 등록 통로가 있을 때 | MAP-03/05/09, LOC-02/03/05, OBS-04/05/06, VEH-01/02, SAFE-02/04/07, SYS-06/07 | 다른 통로가 없음, 오래된 폐쇄 정보, 반복 reroute, local 위험을 해결하지 못함, 시연 map에 갈림길이 없어 장점이 보이지 않음 |
| 제한적 현재 경로 수정 | 기존 경로 주변에 허용된 빈 공간과 안전한 이탈·재합류 구역이 남아 있고 차체가 수행할 수 있을 때 | MAP-04/08, LOC-02/03/05, OBS-02/03/04/06, VEH-01~04, SAFE-02/03/04/07, SYS-06/07 | local minimum·교착, 좁은 공간에서 무리한 통과, 차체 회전·제동 불일치, 움직이는 사람의 상호작용 오판 |
| 조합형 | local 변형 가능한 상황과 topology가 끊긴 상황을 모두 MVP에서 다루기로 할 때 | 위 두 묶음 전부 + local↔global 전환 조건, 계획 validation, route churn 방지, 공통 safety gate | 구현·튜닝·검증 범위 증가, 두 planner의 상태 불일치, 잘못된 전환, 대회 일정 안에 완성도 하락 |

현재 자료에서 도출되는 가장 중요한 부정적 결론은 다음과 같다.

- `정적 장애물 → 전체 재선택`, `동적 장애물 → 국소 수정`처럼 1:1로 고정할 수 없다.
- 멈춘 사람은 곧 움직일 수 있고, 움직이는 병상은 통로 전체를 장시간 막을 수 있다.
- 먼저 정지해야 한다는 기준선은 세 후보 모두에 공통이며 후보 비교 대상이 아니다.
- 경로가 존재한다는 사실은 실제 재출발 허가가 아니다.
- 조합형 사례가 많다는 사실만으로 대회 MVP도 조합형이어야 한다고 결론낼 수 없다.

## 7. 다음 비교 단계에 사용할 고가치 자료 묶음

전부 구현하거나 읽을 필요는 없다. 다음 3후보 비교에서 우선 사용할 핵심 묶음이다.

1. 실제 운영 경계: [WHILL 하네다](https://tokyo-haneda.com/en/service/facilities/whill.html), [요코하마시립시민병원](https://yokohama-shiminhosp.jp/introduction/whill.html), [분당서울대병원 Wheelie](https://www.snubh.org/dh/module/en_ihcstoryView.do?DP_CD=EN&MENU_ID=003011&NO=127&cPage=9)
2. 실제 군중 실패·개선: [Toyota Potaro](https://global.toyota/en/mobility/frontier-research/43981344.html)
3. local/global 경계: [Elastic Bands](https://9p.io/who/seanq/icra93.pdf), [Global DWA](https://cs.stanford.edu/groups/manips/publications/pdfs/Brock_1999_ICRA.pdf)
4. 동적 사람과 sim-real 차이: [Arena-Bench](https://arxiv.org/abs/2206.05728), [Dense Human Crowds](https://journals.sagepub.com/doi/10.1177/0278364914557874)
5. 안전 껍질: [Inevitable Collision States](https://doi.org/10.1163/1568553042674662), [Nav2 Collision Monitor 문서](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)
6. 계획·명령 경계: [Autoware planning design](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/components/planning/), [Planning Validator](https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator/)
7. Python 시험 자산: [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics), [MovingAI](https://www.movingai.com/benchmarks/)
8. Unity·동적 장면: [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub), [Arena-Rosnav](https://github.com/Arena-Rosnav/arena-rosnav), [HuNavSim](https://github.com/robotics-upo/hunav_sim)

## 8. 비교 시험에 남겨야 할 측정 항목

### 안전·정지

- 충돌·접촉 0회 여부
- 최소 이격거리와 near miss
- 위험 관측 뒤 정지 동작 개시와 실제 정지 완료의 구분
- 정지 명령 뒤 추가 이동
- stale pose·obstacle·map·stop state를 주입했을 때 이동을 거부하는지

### 임무·복구

- 목적지 도달 성공
- 전체 재선택 횟수, local 수정 횟수, 원래 경로 재합류 성공
- route churn과 반복 정지·재출발
- 교착·timeout 탐지와 사람 지원 전환
- 정지 원인, 사람 개입률, 평균 복구시간

### 운행 품질

- 경로 길이, 완료시간, 대기시간
- 급가속·급감속·jerk와 불필요한 후진
- 탑승·빈 차 상태의 차이
- 사람에게 예측 가능한 움직임과 사람이 위험하게 양보했는지

어떤 외부 벤치가 충돌을 일부 허용하면서 `success`로 세더라도 우리 합격 기준에는 사용하지 않는다. 실제로 채택하는 각 검증 환경의 결과에는 [VAL-06]이 요구하는 증거 경계를 붙인다.

## 9. 확인하지 못한 것과 다음 단계로 넘길 질문

### 공개 자료에서 확인하지 못한 것

- 병원 전체 임의 위치에서 환자가 앱으로 부르면 빈 자율휠체어가 상시 찾아오는 실제 운영 서비스
- 승객 탑승 병원 휠체어에서 전체 재선택·국소 수정·조합형을 동일 조건으로 비교한 현장 연구
- 상용 서비스의 정지 완료 신호, 자동재개 승인 주체, freshness window와 내부 장애물 분류 규칙
- 우리 축소 차체의 제동거리, 회전 특성, 센서 범위·지연, 계산 주기
- 각 팀원이 실제로 확보할 하드웨어와, 실제로 채택할 시뮬레이션·미들웨어 스택의 지원 버전

### 다음 단계의 비교 질문

1. 대회 MVP map에 실제로 갈림길과 대체 통로를 둘 것인가?
2. local 이탈·재합류를 보여줄 만큼 넓은 구역과 차체 여유가 있는가?
3. 26개 시나리오 중 어떤 상황을 실물 MUST, Python MUST, Unity 설명용으로 둘 것인가?
4. 전체 재선택과 국소 수정 각각이 요구하는 최소 위치 표현은 무엇인가?
5. 입력이 오래됐거나 서로 다른 시점이면 어떤 결과를 폐기하고 정지할 것인가?
6. local 판단에서 global 판단으로 넘어가는 조건을 시간 하나가 아니라 어떤 의미 조건으로 표현할 것인가?
7. 조합형의 추가 구현·시험 비용이 대회의 독창성·완성도 이득보다 큰가?

## 최종 연구 판정

자료 조사만으로 세 후보 중 하나를 채택하지 않는다. 다만 다음 비교 단계의 출발점은 더 명확해졌다.

- 전체 경로 재선택과 제한적 현재 경로 수정은 같은 문제를 푸는 경쟁 알고리즘만이 아니라 서로 다른 공간 변화에 대응하는 기능이다.
- 두 기능을 결합하는 구조는 여러 분야에서 반복되지만, 대회 MVP에 둘 다 넣을지는 시연 map, 차체, 센서 입력, 일정과 검증 비용을 함께 비교해야 한다.
- 어느 후보를 택하더라도 `로봇 로컬 정지 → 실제 정지 확인 → 최신 입력으로 경로 판단 → 별도 안전 게이트의 이동 허가 → 불확실하면 정지 유지` 경계는 바뀌지 않는다.
- 다음 작업은 이 조사 결과와 3단계 의존성 목록을 입력으로 삼아 세 후보를 같은 기준표에서 비교하는 것이다. 구현은 그 뒤다.
