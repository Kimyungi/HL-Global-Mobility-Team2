# v2_main 수식 보존 최적화: 2차 결과

운영 코드 변경 없이 오프라인 시제품을 더 최적화했다. 이전 1.6~1.7배 대비 전체 콜백 기준 3.2~5.6배를 확인했다. 이 수치는 합성 장면의 같은 실행 내 기준 구현/시제품 비교이며 실제 주행 원시 scan 재생 결과가 아니다.

| 장면 | 기준 구현 중앙값 | 새 시제품 중앙값 | 속도비 | 캐시가 빈 첫 실행 |
| --- | --- | --- | --- | --- |
| 범퍼 앞 3.3m 물체 | 20.99ms | 6.47ms | 3.25배 | 9.76ms |
| 범퍼 앞 2.0m 물체 | 23.39ms | 6.97ms | 3.36배 | 11.25ms |
| 범퍼 앞 0.7m 물체, 경로 실패 | 225.64ms | 40.10ms | 5.63배 | 60.50ms |

각 중앙값은 최초 실행을 제외한 6회다. 이전 시제품의 같은 실패 장면 측정은 139.42ms였다(별도 실행). 수치 원본은 fast_native_results.json에 있다.

## 유지한 내용

GPS station+1m anchor, 회피점 투영 station+2.7m 복귀점, cubic Hermite와 도함수/곡률 수식, 0.05m 샘플 간격, 샘플 수 공식, 4개 접선 scale 후보, 후보 순서, 회전반경, 차체 여유, 장애물 충돌 조건을 유지했다. 후보를 줄이거나 충돌 검사를 생략하지 않았다.

## 변경한 실행 방법

- fast_cubic_v2.py: Python에서 같은 수식으로 계산한 기저계수를 count별 재사용한다. 좌표·도함수의 4항 내적을 NumPy 배열 연산으로 묶되 덧셈 순서를 유지한다. yaw, hypot, 곡률 산출은 기존 Python math 및 스칼라 식을 유지한다. 배열 cache와 원본 계수 cache는 각각 128개로 제한했다.
- footprint_native.cpp / native_footprint.py: 회전된 차체 상자와 장애물 선분의 기존 slab 교차 판정을 C++ 반복문으로 실행한다. sin/cos는 기존 NumPy로 계산한 값을 넘긴다. float64와 동일 비교식을 사용하며 -ffp-contract=off, fast-math 없음으로 컴파일했다. Python과 NumPy의 반복적인 임시 배열 생성 부담을 줄인다.
- 충돌 검사 묶음을 단순 확대한 fast_footprint.py는 실패 장면에서 느려져 최종 조합에서 제외했다. 관련 benchmark_fast_combined.py와 측정 JSON은 비교 기록이며 적용 대상이 아니다.

## 동치 검증

1. 고정 seed 난수와 좌우/scale 조합 총 258개 입력에서 생성된 27,822개 점의 x/y/yaw/curvature가 기준과 float64 비트 단위 일치.
2. 충돌 판정 1,500개 난수 장면과 접촉 경계/평행 선분/단일점/경계 양쪽 한 ULP 사례 24개, 총 1,524개 결과 일치.
3. 세 성능 측정 장면의 전체 경로, 제어 목표점, 실패 사유 일치.
4. 최적화 함수를 임시 대입한 stack_avoid/test 전체 154개 통과(1.10초).

이는 시험한 입력에 대한 확인이며 모든 가능한 부동소수점 경계의 보편적 동치 증명은 아니다. 입력 검증과 기존 freshness 규칙을 운영 통합 시에도 유지해야 한다.

## 재현

저장소 루트에서 아래 명령으로 컴파일 및 합성 벤치마크를 실행한다. 차량/CAN을 실행하는 명령은 없다.

```bash
g++ -O3 -std=c++17 -shared -fPIC -ffp-contract=off \
  analysis/run_20260914_212415_diagnosis/footprint_native.cpp \
  -o analysis/run_20260914_212415_diagnosis/footprint_native.so
source /opt/ros/humble/setup.bash
source install_v2/local_setup.bash
PYTHONPATH="src/stack_avoid:src/stack_avoid/test:$PYTHONPATH" \
  python3 analysis/run_20260914_212415_diagnosis/benchmark_fast_native.py
```

운영 반영 시에는 분석 폴더의 .so 경로를 하드코딩하지 않고 패키지 빌드/설치에 네이티브 모듈을 포함해야 한다. 모듈 미설치/ABI 오류의 처리를 명시해야 한다. 센서 발행과 동기 계산이 묶인 구조, 현장 장면의 최대 계산 시간은 별도 확인 대상이다. 이 합성 실패 장면은 캐시가 빈 경우에도 100ms 아래지만 실제 주행 2.27~3.55초 공백의 해소를 보장하지 않는다.
