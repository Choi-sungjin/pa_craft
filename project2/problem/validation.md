# Q1 할당 검증

제공된 `validate_problem.py`와 `example-problem.json`을 변경 없이 가져와 실행함. `example-input.json`은 제공된 JSON의 사본임. 실행 결과는 [validation.json](validation.json)에 기록함.

```bash
python3 project2/problem/validate_problem.py project2/problem/example-input.json
```

제공 검증기는 12개 고유 ID, 색상·모양, R/Y/G 횟수 범위와 총합, Y의 동일 모양, 남은 R/G 수량을 검사함. Q1에서 Y=[0,1], R=[2,3,8,9], G=[4,5,6,7,10,11]인 전체 할당을 찾음.

정리 과정에서 신호 순서와 횟수 일치, 배치표의 12개 ID, 정답의 중복·누락, 60 mm 격자 좌표도 별도로 대조함. 제공 검증기가 신호·바코드·격자 좌표까지 검사한다고 해석하지 않음.

결과는 **문제의 조합적 풀이 가능성**을 의미함. 로봇의 도달 거리, 충돌, 실물 배치 오차, 실제 경기 성공, 다른 팀 검토를 입증하지 않음. 제공된 예제 조합 목록을 다른 색상·모양 구성 전체에 일반화하지 않음.
