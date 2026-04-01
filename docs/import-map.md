# 브랜치 Import 맵

`main` 브랜치에서 아래 원격 추적 브랜치를 히스토리 보존 방식으로 prefix import 했습니다.

## 소스 브랜치 -> 디렉터리

- `origin/server` -> `server/`
- `origin/test/Tank/merge` -> `tank/`
- `origin/ESP32-S3_meri` -> `ruview/`
- `origin/QtProgramming` -> `qt/`

## 메인 브랜치 import 커밋

- `0949750`: `origin/server` import
- `a32c08b`: `origin/test/Tank/merge` import
- `0107e2e`: `origin/ESP32-S3_meri` import
- `becb745`: `origin/QtProgramming` import

## 메모

- 루트 placeholder였던 `test.txt`는 통합 정리 단계에서 제거했습니다.
- 이후 커밋에서는 폴더 구조, `CMake`, 설정 파일, `README`, 문서를 통합 구조에 맞게 다시 정리합니다.
