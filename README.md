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
