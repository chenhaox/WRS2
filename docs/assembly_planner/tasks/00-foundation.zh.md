# 00：输入、数据契约和无界面运行基础

状态：待实施。前置：无。里程碑 M1 的起点。

## 目标和修改范围

建立一个能在 Python 3.12 + NumPy/SciPy 环境独立导入、验证输入和运行小型测试的 `wrs.assembly`。实现 [contracts](../contracts.zh.md) 中本阶段需要的核心模型；后续结果类型至少固定字段语义和协议，不写虚假的成功实现。

负责 `wrs/assembly/model.py`、`io.py`、`adapters/legacy.py`、轻量包入口、`tests/assembly/fixtures.py`、`test_io.py`。为解决父包 eager import，可有针对性地修改 `wrs/__init__.py` 并保留现有公开入口兼容性。必要的依赖/fixture ignore 例外归本任务管理。

## 实施步骤

1. 检查当前分支、基线、已存在文件和 `docs/COMMENT_STYLE.md`，将接口约定落实为唯一的数据类型定义。数组复制/只读、输入校验、稳定 ID 和几何摘要必须实际实现。
2. 定义版本化 manifest，明确 mesh 单位与 pose；输入需要质量/COM 时缺失即报告 unknown，不隐式补 1 kg。输出 canonical 的米制数组和 provenance。
3. 用已有 WRS loader 的逻辑/可复用底层能力读取 STL，保留分析精度与原始面 ID；不要先经固定精度的顶点焊接再声称微小间隙保真。不为分析数据创建 SceneObject 或 collision shape。
4. 解决 `import wrs.assembly` 会先执行 WRS 根包 eager imports 的问题。必要时采用兼容的延迟导出；只延迟 heavyweight 路径，不改变公开名字和对象含义。
5. 编写确定性的箱体、方环、双连通面、简单圆柱/圆孔、解析平面支撑 fixture。将尺寸、公差和解析预期写明，不启动 viewer。
6. 为旧 `domino_5`、`burrpuzzle` 输出资产清单、实例 pose、单位/rotation 映射核验报告。输入来源优先用户完成克隆后的 `asp_old/assembly_planner`，否则使用本计划列出的参考副本并记录 commit。bridge 缺件要如实记录。

## 验收

- mm 和 m 两份同一模型经导入产生相同 canonical 几何和 pose；旋转用非交换的三轴例子核验旧 convention。
- 非有限值、非法三角索引、坏矩阵、重复实例 ID、未声明 STL 单位得到可读错误。
- 同一 mesh 的两个零件实例可以有不同 pose；改变外部输入数组不会改变已构建模型。
- 没有 MuJoCo、CAD 和显示环境时可以导入分析模块并运行 `test_io.py`。
- 在相应依赖齐全的环境验证已有 `from wrs import wvw, wsso, wssop, Grasp, MotionData` 等公共导出；未具备环境的检查明确记未测，不伪装成兼容性验证通过。
- 交付供后续任务使用的 fixture API、数据 schema 示例和实际运行命令。

## 可复制到新对话的提示词

```text
在 D:\code\ch\asp\WRS2 的 codex/assembly-planner 开发分支实施任务 00。
先读 docs/assembly_planner/README.zh.md、contracts.zh.md 和 tasks/00-foundation.zh.md，以及仓库适用说明。
这次只建立数据契约、输入/单位、旧数据清单、无界面导入边界与 fixture，不实现 contact detector 或 planner。
检查当前工作区并保留已有修改；未来 API 以 contracts 为单一约定。核心测试不能依赖 Panda3D、MuJoCo、viewer 或 CAD。
完成实现与本任务验收，记录实际命令/输出、修改文件、契约版本、可供下一任务复用的基线。将交接结果附在本任务文档末尾，不声称未运行的集成测试通过。
```

## 交接记录

实施后填写：基线/完成 commit、测试命令与输出、schema/fixture API、兼容性验证范围、未解决事项。当前未实施。
