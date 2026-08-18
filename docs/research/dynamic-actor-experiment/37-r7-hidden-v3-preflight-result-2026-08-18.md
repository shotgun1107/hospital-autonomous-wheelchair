# R7 hidden-v3 사전점검 결과

## 1. 결론

hidden-v3 실행 도구와 판정 규칙을 수정했고, seed를 만들지 않는 사전점검까지 통과했다.

```text
runner 준비: 완료
전체 회귀: 통과
clean-clone preflight: 통과
hidden seed 생성: 안 함
hidden 20개 실행: 안 함
```

실제 hidden-v3 실행은 사용자의 별도 승인을 기다린다.

## 2. 구현

구현 commit은 `6a272ff223098436f1b2010d8eaed4efbb36fccc`다.

- schema를 `r7-hidden-observation-v2`로 올렸다.
- case ID를 `hidden-v3-*`로 바꿔 과거 결과와 섞이지 않게 했다.
- Normal은 실제 이동, Actor 통과, 계획 정지 뒤 원 경로용 새 실행, 목적지 도착 순서를
  요구한다.
- Normal·Stress 모두 Actor와 정적 장애물 여유 `0.08 m` 이상을 요구한다.
- v4 증거 ZIP의 크기·SHA-256·receipt·5/5 동일성·0/500 시간 자격을 확인한다.
- 공식 DLL 해시와 현재 DLL 해시가 다르면 실행 전에 중단한다.
- `--preflight-only`는 seed와 case를 만들지 않는다.
- 사용자가 root seed를 넣는 옵션은 없다.
- 중단이나 실행 오류는 부분 결과와 별도의 오류 파일로 남기고 알고리즘 FAIL과 구분한다.

## 3. 검증

직접 영향 시험은 `27 passed`였다. Ruff, compileall과 diff 검사도 통과했다.

전체 85개 시험 파일은 14개 process로 나눠 실행했다.

- 병렬 결과: `981개 중 980 pass`, 시간측정 시험 `1 fail`
- 실패 이유: 병렬 CPU 경쟁 중 내부 60초 제한을 `72.423초`로 초과
- 규칙에 따라 해당 시간측정 시험만 단독 재실행: `1 passed in 279.47s`
- 최종 판정: 고유 시험 `981/981 pass`

병렬 부하에서 나온 시간 초과를 알고리즘 실패로 사용하지 않았고, 시간측정은 단독 실행 결과만
판정에 사용했다.

## 4. seed 없는 사전점검

전용 깨끗한 복제본에서 다음 결과를 확인했다.

| 항목 | 결과 |
|---|---|
| HEAD | `6a272ff223098436f1b2010d8eaed4efbb36fccc` |
| tree | `ab555482ab4196eb606dadd808de2c24c12237a0` |
| worker 예정값 | `14` |
| R7 parity | `5/5` |
| 50 ms 초과 | `0/500` |
| evidence ZIP | `7,773 bytes`, `3829e14d...6183` |
| R7 receipt | `a971ffef...ef64` |
| hidden seed 생성 | `false` |
| hidden 실행 | `false` |

사전점검 산출물은 회사 PC의 다음 임시 복제본에 있다.

```text
C:\Users\SSAFY\AppData\Local\Temp\hospital-hidden-v3-preflight-c6e7d656469f4ea0a067cf7977c3ea92
```

`preflight-manifest.json` 하나만 생성됐으며 seed 관련 파일은 생성되지 않았다.

## 5. DLL 재빌드 주의사항

동일 소스에서 DLL을 새 임시 경로로 다시 빌드했지만 파일 SHA-256은 공식 자격 빌드와 달랐다.

| 파일 | 새 재빌드 | 공식 자격 빌드 |
|---|---|---|
| `dwb_full_core.dll` | `fd5cbf1b...3a16` | `236be945...fd67` |
| `dwb_safety_core.dll` | `89b79aa0...6ae` | `c99ee4a5...961f` |

소스 변경이 아니라 빌드 경로 등 DLL 내부 정보의 차이로 보인다. 이번 preflight는 R7 0/500 자격에
실제로 사용된 공식 DLL을 보존된 자격 복제본에서 가져와 해시를 확인한 뒤 실행했다.

따라서 actual hidden도 위 preflight 복제본과 공식 DLL을 그대로 사용해야 한다. 새로 빌드한 DLL로
바꾸려면 먼저 새 R7 500회 자격을 다시 받아야 한다.

## 6. 다음 작업

1. 사용자에게 actual hidden-v3 한 번 실행 승인을 받는다.
2. 승인되면 위 clean clone에서 새 output 경로를 사용한다.
3. 다른 simulation·시간측정 작업이 없는지 확인한다.
4. 새 63-bit seed를 내부 생성하고 20개를 14 process로 실행한다.
5. PASS·FAIL·중단 결과를 그대로 보존하고 같은 실행에서 코드를 고치지 않는다.

이 결과는 합성 simulation 실행 준비 증거다. 실제 카메라, 실제 사람, 제품 알고리즘 채택이나
사람 탑승 안전 증거가 아니다.
