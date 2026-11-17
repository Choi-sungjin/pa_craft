# 오류·미완료 추적

아래 GitHub Issue와 문서에서 발견한 항목을 구분함. 문서상 항목을 실제 등록된 GitHub Issue로 표시하지 않음.

| 구분 | 항목·발생 조건 | 원인·상태 | 담당·검토자·근거 |
| --- | --- | --- | --- |
| 기존 Issue | [#19 same size_t plane](https://github.com/Choi-sungjin/pa_craft/issues/19) | GitHub에서 closed/completed 상태 확인됨. 과거 TensorRT 코드 관련 항목임 | 담당자 미지정. LM 실패와 별개임 |
| 문서 추적 P2-01 | ObservePucks / PlacePuck / VerifyLocation 실행 | skeleton이 시간 경과로 성공 반환함. 미구현임 | 담당·검토자 미확인. architecture.md에 소스 연결함 |
| 문서 추적 P2-02 | 12개 실제 ID·속성 등록 | `assume_unknown_picks: true`, 실제 관측 연결 미완료임 | 담당·검토자 미확인 |
| 문서 추적 P2-03 | 전체 할당 수행 시점 | 현재 Collect 이후 Allocate임. 발제의 전체 할당 선행 흐름과 차이 있음 | 담당·검토자 미확인 |
| 문서 추적 P2-04 | integ21 / S3 두 번째 집기 | LM 도달 범위 밖 판정. 근본 원인은 추가 확인 필요함 | [실패 로그](../results/development/integ21_1116_1429/picks/pick_1794807562105598306.log) |
| 문서 추적 P2-05 | 문제지 좌표와 운영 지도 적용 | 좌표·도킹 기준 정합 근거가 부족함 | [출처 검토](source-review.md) |
| 문서 추적 P2-06 | 최종 제출 증빙 | 공식 두 회차·개인별 기여·재현·다른 팀 검토·영상 공개 링크 미확인임 | 각 제출 폴더의 상태 기록 참조함 |

새 Issue 생성이나 담당자 배정은 수행하지 않았음. 실제 등록 시 발생 조건·원인·수정자·검토자·수정 커밋·재시험 결과를 연결하면 됨.
