# 10：集成、可复现基准和使用入口

状态：M2/M3 使用入口、分析器基准、独立进程核心依赖隔离和机器人端到端回归已实现（2026-09-11）。旧 domino/burr 数据完整迁移和 CAD 对照仍待实施。详见 [交接](../m2-m3-execution.zh.md)。

## 目标和修改范围

让新对话或新环境能够按命令重现分析与装配结果，建立长期回归基线，并用证据区分 M1/M2/M3 的完成范围。

负责 `tests/assembly/test_end_to_end.py`、`examples/assembly/analyze_contacts.py`、`plan_assembly.py`、可复现 benchmark 入口和使用/结果文档。整合必要的 dispatcher、API 索引和窄范围 fixture ignore 例外。

## 实施步骤

1. 为纯 contact 分析提供 manifest -> report/overlay 的命令入口；为规划提供 manifest -> sequence/state/path report 的入口，参数包含单位、公差、seed 和预算。
2. 组织三层测试：解析几何真值、真实 mesh/CAD 几何对照、端到端装配/机器人 replay。测试既包括应通过案例，也包括必须失败或 unknown 的案例。
3. 首批旧数据使用 `domino_5`（实际 3 件）和 `burrpuzzle`（6 件）。核验单位、Euler convention、质量/COM/摩擦后再评价。bridge 的 alframe.stl 补齐之前报告 missing_asset，不自动替换成近似形状。
4. 旧 planner 的接触或序列结果只作对照；不要把旧算法所有判断都设为新实现的 golden truth。用新几何证据解释差异。
5. 性能记录机器/依赖、网格大小、候选数、距离查询、预算、冷/热缓存、耗时和峰值内存；测量后再制定回归阈值。
6. 跨 seed/三角化/整体刚体变换/单位转换/公差扫描检查结果稳定性；曲面显示随细分收敛和 unresolved 比例。
7. 全部验收使用指定解释器 `D:\code\venv312\.venv\Scripts\python.exe`。对 M1/M2 用隔离子进程阻断可选模块导入，验证核心依赖边界；使用同一解释器的完整依赖运行 08 的演示。实际产物与 schema 版本一同保存，输出文件可重建。
8. 更新 API 索引和简洁教程，检查 examples/fixtures 不依赖旧电脑绝对路径。旧源码路径只允许作为显式 migration 参数。

## 完成标准

- 一条命令完成小 fixture 的 contact report，面积、法线场、gap、孔和分类与解析预期相符。
- 一条命令完成至少一个有真实几何与静力验证的小型装配序列，能独立 replay。
- M3 若声明完成，必须有 WRS 正向执行、持物与夹爪碰撞、IK、支撑切换的实际证据。
- 没有可运行的旧案例、CAD 或 GUI 时，逐项标明缺失，不能把 skip 计入成功率。
- 用户能只凭仓库文档重跑；所有 fixture、配置、seed、版本和命令可追溯。

## 可复制提示词

```text
在 D:\code\ch\asp\WRS2 实施 docs/assembly_planner/tasks/10-validation.zh.md。
统一使用 D:\code\venv312\.venv\Scripts\python.exe；安装依赖用该解释器的 -m pip，先核对 sys.executable，不另建虚拟环境。
先读 README.zh.md、contracts.zh.md 和已完成任务的交接，按实际状态区分 M1/M2/M3；不要把可选或未完成能力算成功。
建立 analyze_contacts / plan_assembly 使用入口、解析/真实 mesh/端到端回归及可复现性能报告。迁移 domino_5、burrpuzzle 时先核验单位和物理参数，bridge 缺件保持明确失败。
旧算法是对照而非真值，必须保留孔、间隙、法线场、穿透和 unknown 的验证。记录指定解释器的运行命令、核心依赖隔离检查、版本、seed、实际输出及局限，补齐 API 索引和使用教程。
```

## 交接记录

已交付 `plan_assembly.py` manifest 入口、`sequence_demo.py`、`robot_execution_demo.py`、`benchmark_execution.py` 和 `test_end_to_end.py`。M2/M3 命令、版本、原始基准 JSON 和未实现范围见 [交接](../m2-m3-execution.zh.md)。旧数据的物理参数尚未完成核验，未把旧结果作 golden truth，也未宣称完成 CAD 验收。
