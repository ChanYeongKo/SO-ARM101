# Project B - 단일 뎁스카메라 구성

> D405 대신 1m+ 측정 가능한 뎁스카메라 1개로 단순화한 구성

---

## Project A vs Project B 비교

| 항목 | Project A (D405 손목) | Project B (1m+ 탑 고정) |
|------|----------------------|------------------------|
| 카메라 수 | 2개 (탑 RGB + 손목 D405) | **1개 (탑 RGB-D)** |
| 장착 위치 | 손목 (팔과 같이 이동) | **차량 상단 (고정)** |
| 캘리브레이션 | 핸드-아이 (복잡) | **고정 위치 1회만** |
| 근거리 정밀도 | 매우 높음 | 보통 |
| 구현 난이도 | 높음 | **낮음** |
| 추천 대상 | 정밀 파지 | **학부 캡스톤** |

---

## 전체 시스템 블럭도 (2학기 최종)

```mermaid
flowchart TD
    subgraph SENSOR["센서 레이어"]
        DC["탑 뎁스카메라\n(RGB-D, 1m+)\n차량 상단 고정"]
    end

    subgraph PERCEPTION["인식 레이어"]
        YO["YOLOv8\n쓰레기 감지"]
        PO["3D 좌표 변환\nCamera Intrinsics\n픽셀 + 깊이 → X,Y,Z"]
        PC["Point Cloud 분석\n물체 크기 측정"]
        TF["고정 좌표 변환\n카메라 → 로봇 베이스\n(1회 캘리브레이션)"]
    end

    subgraph DECISION["판단 레이어"]
        GD["Garbage Detector Node\n(ROS2)"]
        NAV["Navigation2\n경로 계획"]
        IK["IKPy\n역기구학 계산\nX,Y,Z → θ1~θ6"]
        GR["그리퍼 개도 계산\n물체 폭 × 1.2"]
    end

    subgraph ROBOT["실행 레이어"]
        TB["TurtleBot3 Waffle\n자율주행"]
        ARM["SO-ARM101\n로봇팔"]
        LR["LeRobot\n서보 제어"]
    end

    subgraph OUTPUT["결과"]
        BIN["쓰레기통\n(지정 위치)"]
    end

    DC -->|"RGB 영상"| YO
    DC -->|"Depth 맵"| PO
    DC -->|"Point Cloud"| PC

    YO -->|"쓰레기 BBox (u,v,w,h)"| PO
    PO -->|"X,Y,Z (카메라 좌표계)"| TF
    PC -->|"물체 폭 (mm)"| GR
    TF -->|"X,Y,Z (로봇 좌표계)"| GD

    GD -->|"이동 목표"| NAV
    GD -->|"집기 목표 위치"| IK
    GR -->|"그리퍼 각도"| IK

    NAV -->|"cmd_vel"| TB
    TB -->|"근접 정지"| GD
    IK -->|"관절 각도 θ1~θ6"| LR
    LR -->|"서보 제어"| ARM
    ARM -->|"집기 완료"| NAV
    NAV -->|"쓰레기통으로 이동"| TB
    TB -->|"도착"| IK
    ARM -->|"투입"| BIN
```

---

## 1학기 블럭도

```mermaid
flowchart TD
    subgraph CAM["탑 뎁스카메라 (RGB-D, 차량 상단 고정)"]
        RGB["RGB 영상"]
        DEPTH["Depth 맵"]
        PCL["Point Cloud"]
    end

    subgraph SW["Host PC (Ubuntu 24.04 / ROS2 Jazzy)"]
        YO["YOLOv8\n쓰레기 감지\nBBox(u,v,w,h)"]
        COORD["3D 좌표 변환\nCamera Intrinsics\nX,Y,Z (카메라 좌표계)"]
        TF["고정 좌표 변환\nX,Y,Z (로봇 베이스 좌표계)\n1회 캘리브레이션으로 고정"]
        SIZE["Point Cloud 분석\n물체 폭 측정 (mm)"]
        GR["그리퍼 개도 계산\n개도 = 물체 폭 × 1.2"]
        IK["IKPy 역기구학\n목표 위치 → 관절 각도\nθ1 ~ θ6"]
    end

    subgraph HW["로봇팔 하드웨어"]
        LR["LeRobot\n서보 드라이버"]
        ARM["SO-ARM101\n로봇팔 (Follower)"]
    end

    subgraph RESULT["결과"]
        GRAB["쓰레기 집기 완료"]
    end

    RGB -->|"영상 스트림"| YO
    DEPTH -->|"픽셀별 깊이값"| COORD
    PCL -->|"3D 포인트"| SIZE
    YO -->|"쓰레기 중심 픽셀"| COORD
    COORD -->|"X,Y,Z (카메라 기준)"| TF
    SIZE -->|"물체 폭"| GR
    TF -->|"X,Y,Z (로봇 기준)"| IK
    GR -->|"그리퍼 각도"| IK
    IK -->|"관절 각도 θ1~θ6"| LR
    LR -->|"서보 모터 제어"| ARM
    ARM --> GRAB
```

---

## 전체 동작 흐름

```mermaid
sequenceDiagram
    participant CAM as 탑 뎁스카메라 (RGB-D)
    participant AI as YOLOv8
    participant GD as Garbage Detector
    participant IK as IKPy
    participant NAV as Navigation2
    participant CAR as TurtleBot3
    participant ARM as SO-ARM101

    CAM->>AI: RGB 영상 스트림
    AI->>GD: 쓰레기 BBox + 클래스
    GD->>GD: Depth로 쓰레기 3D 좌표 계산
    GD->>GD: Point Cloud로 물체 폭 측정
    GD->>NAV: 쓰레기 위치로 이동 명령
    NAV->>CAR: 경로 생성 및 이동
    CAR->>GD: 근접 정지 (1m 이내)

    GD->>GD: 3D 좌표 저장 (팔이 가려도 유지)
    GD->>IK: 목표 위치 (X,Y,Z) + 물체 폭 전달
    IK->>IK: 관절 각도 계산 (θ1~θ6)
    IK->>IK: 그리퍼 개도 계산
    IK->>ARM: 관절 각도 명령 (LeRobot)
    ARM->>GD: 집기 완료

    GD->>NAV: 쓰레기통 위치로 이동 명령
    NAV->>CAR: 경로 생성 및 이동
    CAR->>GD: 도착 완료
    GD->>IK: 투입 위치 전달
    IK->>ARM: 관절 각도 명령
    ARM->>GD: 투입 완료
```

---

## 핵심 포인트 - 가림 현상 해결

탑 카메라가 고정이므로 팔이 움직이면 카메라 시야를 가릴 수 있습니다.

```mermaid
flowchart LR
    A["터틀봇 정지"] --> B["3D 좌표 측정\n및 저장"]
    B --> C["팔 이동 시작\n(카메라 가려도 OK)"]
    C --> D["저장된 좌표로\nIKPy 계산"]
    D --> E["집기 완료"]
```

> 터틀봇이 멈춘 직후 좌표를 한 번만 저장하면, 이후 팔이 카메라를 가려도 문제없습니다.

---

## 추천 뎁스카메라

| 카메라 | 측정 거리 | ROS2 지원 | 특징 |
|--------|----------|----------|------|
| Intel RealSense D435i | 0.3m ~ 3m | ✅ 공식 | 가장 무난 |
| Intel RealSense D455 | 0.6m ~ 6m | ✅ 공식 | 넓은 시야각 |
| OAK-D (Luxonis) | 0.2m ~ 수m | ✅ 공식 | AI 연산 내장 |

---

## 기술 스택

| 구분 | 기술 | 역할 |
|------|------|------|
| OS | Ubuntu 24.04 | 개발 환경 |
| 로봇 미들웨어 | ROS2 Jazzy | 노드 간 통신, TF 관리 |
| 자율주행 | TurtleBot3 Waffle + Navigation2 | 자율 이동 및 경로 계획 |
| 로봇팔 | SO-ARM101 (Follower) | 쓰레기 집기 실행 |
| 뎁스카메라 | RGB-D 카메라 1m+ (차량 상단) | 감지 + 3D 위치 + 크기 측정 |
| 물체 인식 | YOLOv8 | 쓰레기 감지 및 BBox 추출 |
| 팔 제어 | IKPy (역기구학) | 위치 독립적 관절 각도 계산 |
| 서보 제어 | LeRobot (feetech) | SO-ARM101 서보 드라이버 |
| 언어 | Python | - |

---

## 개발 로드맵

### 1학기 - 로봇팔 위주
- [ ] 개발 환경 구축 (ROS2 Jazzy + IKPy + LeRobot)
- [ ] SO-ARM101 캘리브레이션 및 기초 제어
- [ ] 뎁스카메라 ROS2 연동 및 고정 좌표 캘리브레이션
- [ ] YOLOv8 쓰레기 감지 + 3D 좌표 추출
- [ ] Point Cloud로 물체 크기 측정 → 그리퍼 개도 계산
- [ ] IKPy 역기구학 구현
- [ ] 통합 테스트 (감지 → IK → 집기 자율 동작)

### 2학기 - 전체 통합
- [ ] TurtleBot3 자율주행 연동
- [ ] 전체 파이프라인 통합 테스트
- [ ] 성능 최적화 및 시연
