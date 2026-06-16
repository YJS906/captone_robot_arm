# OpenManipulator Joint2 Limit 변경

본 프로젝트에서는 OpenMANIPULATOR-X가 낮은 위치의 목표물을 집기 위해 `joint2`의 MoveIt/URDF 제한을 넓혀 사용했습니다.

환경에 따라 아래 파일을 수정해야 합니다.

```text
open_manipulator/open_manipulator_x_description/urdf/open_manipulator_x.urdf.xacro
```

기존:

```xml
<limit velocity="4.8" effort="1" lower="${-1.5}" upper="${1.5}" />
```

변경:

```xml
<limit velocity="4.8" effort="1" lower="${-1.5}" upper="${2.05}" />
```

수정 후 OpenManipulator 관련 패키지를 다시 빌드해야 합니다.
