# CCTV SUMO 교통 시뮬레이션

SUMO와 TraCI 브리지를 이용해 CCTV별 교차로 시뮬레이션을 실행하고, 백엔드의 차량 수와 신호 제어 결과를 적용하는 프로젝트입니다.

## 실행 환경

- Windows
- Eclipse SUMO 및 SUMO GUI
- Python 3
- Python 패키지 `requests`
- Python에서 SUMO의 TraCI 모듈을 import할 수 있도록 SUMO tools 경로 설정

```powershell
python -m pip install requests
```

## CCTV별 실행

프로젝트 폴더에서 PowerShell을 열고 실행합니다.

```powershell
python bridge.py
```

위 명령은 CCTV1을 실행합니다. CCTV2와 CCTV3은 각각 다음 명령으로 실행합니다.

```powershell
python bridge_cctv2.py
python bridge_cctv3.py
```

각 브리지는 해당 CCTV의 `data` 하위 SUMO 설정과 네트워크를 사용합니다. 백엔드 주소와 `intersection_id`는 각 브리지 파일 상단에 설정되어 있습니다.

## 프로젝트 구성

- `bridge.py`: CCTV1 SUMO 실행, 백엔드 응답 처리, 차량 투입 및 신호 제어
- `bridge_cctv2.py`, `bridge_cctv3.py`: CCTV2·CCTV3 전용 브리지
- `data/backend_demo.sumocfg`, `data/cross.*`: CCTV1 SUMO 설정과 네트워크 원본 파일
- `data/cctv2/`, `data/cctv3/`: CCTV2·CCTV3 별도 설정과 네트워크 파일
- `runner.py`: 기본 SUMO/TraCI 예제 실행 파일

## 응급차량 시연 참고

CCTV1 브리지는 `GET /api/control/signal?intersection_id=1`에서 `result.event_state`, `priority_direction`, `emergency_vehicle_id`, `green_times`를 읽습니다. 응급 이벤트는 기존 신호 황색 전환을 유지하면서 우선 방향을 열고, `CLEARED` 수신 후 저장한 신호 단계로 복귀합니다. 응급 이벤트 전환 황색은 3초이며, 일반 신호 단계의 황색 시간은 네트워크 설정을 따릅니다.

현재 브리지 파일에는 백엔드 주소가 설정되어 있습니다. 팀 환경에서 주소가 달라지면 각 브리지의 `BACKEND_URL`을 맞춰야 합니다. 주소가 사설 네트워크이므로 이 저장소는 팀원만 접근할 수 있는 비공개 저장소로 공유하는 것을 권장합니다.

## CCTV1 BASELINE / ADAPTIVE 비교

비교 실행기는 실시간 CCTV1 브리지와 별도 파일로 동작합니다. 백엔드에서 차량 배치를 받은 뒤, 같은 배치를 BASELINE과 ADAPTIVE에 넣어 비교합니다. 결과는 `data/comparison_results/` 아래에 저장됩니다.

```powershell
python comparison_runner.py --mode both
```

모든 모드는 SUMO를 실행하기 전에 유효한 새 백엔드 차량 배치를 기다립니다. 백엔드가 이미 `ACTIVE`인 채로 시작하면 이전 응답일 수 있어, `WAITING -> ACTIVE` 전환이나 방향별·회전별 차량 수 변경까지 SUMO를 시작하지 않습니다. 현재 `ACTIVE` 응답을 의도적으로 바로 사용하려면 `--accept-current`를 붙입니다. `--mode both`는 같은 입력을 BASELINE과 ADAPTIVE로 각각 실행합니다. `--gui`를 붙이면 SUMO-GUI로 실행하고 기본적으로 초당 한 시뮬레이션 초의 속도로 진행합니다. 속도는 `--gui-delay-ms`로 조정할 수 있습니다.

```powershell
python comparison_runner.py --mode both --duration 180 --warmup 20
```

차량 수와 회전 비율은 백엔드의 검증된 `data`에서 가져옵니다. 비교 실행기는 방향별 총 차량 수와 좌·직·우회전별 차량 수가 일치하지 않거나 녹색 시간이 유효하지 않으면 실행하지 않습니다. `comparison_scenario.json`은 최대 실행 시간, 워밍업, 초기 동쪽 대기열, 투입 간격과 seed만 설정합니다. 응급차는 백엔드 응답에 ACTIVE 이벤트가 있을 때만 같은 방향에서 추가합니다.

BASELINE은 캡처한 차량 배치를 사용하면서 네트워크 기본 신호 주기를 유지합니다. ADAPTIVE는 같은 차량 배치를 다시 넣고 캡처한 백엔드 녹색 시간을 TraCI로 적용합니다. 각 실행은 예정 차량과 도로 위 차량이 모두 사라지면 조기 종료하며, 다음 백엔드 차량 정보가 도착할 때까지 기다립니다. 차량 정보가 설정된 idle timeout 동안 바뀌지 않으면 프로그램을 종료합니다. 방향별 차량 수가 같더라도 새 배치인지 구분하려면 백엔드 응답에 `batch_id` 또는 `frame_id`를 포함하는 편이 안전합니다.

현재 비교 실행기는 BASELINE의 정상 신호 시간을 백엔드에 요청하지 않고 SUMO 네트워크 기본값을 사용합니다. BASELINE과 ADAPTIVE 신호 시간을 모두 백엔드가 계산하도록 하려면, 백엔드가 동일한 `batch_id`에 대해 프로필별(`baseline` / `adaptive`) 녹색 시간을 반환하는 API 규격을 추가하고 실행기에서 해당 API를 호출하도록 연결해야 합니다.

각 배치는 원본 백엔드 응답 JSON과 모드별 tripinfo XML, 1초 간격 대기열·정지시간 CSV, 차선별 E2 검지기 XML, 차선별 요약 CSV를 생성합니다. 두 모드의 비교 요약은 `data/comparison_results/<실행시각>/batch_NNN/comparison_summary.json`에 저장됩니다. 배치에 포함된 도로 차량과 예약 차량이 모두 사라지면 해당 SUMO 세션을 끝내고 다음 새 차량 배치를 기다립니다. 추가 배치가 기본 5초 동안 오지 않으면 실행기를 종료하며, 대기 시간은 `--idle-timeout`으로 바꿀 수 있습니다.
