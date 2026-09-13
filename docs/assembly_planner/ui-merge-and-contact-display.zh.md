# 主分支 UI 合并与示例接触面显示

> 入口整理说明（2026-09-13）：本文的旧脚本命令、输出路径和计时保留为历史记录。当前请按 [WRS 例子使用说明](../../examples/assembly/README.md) 运行，参数在文件顶部修改；benchmark 已移出 examples。

2026-09-11。当前开发分支仍为 `codex/assembly-planner`。

## 文件保存与合并

- 合并前还有 29 个未提交源码、测试、示例和文档，已整体提交为 `7e73d29`。
- `origin/main` 与 `upstream/main` 均为 `2bb014b`，包含 `e3358ee` 的 Web UI 控件更新。
- 合并提交为 `f65607c`。唯一冲突是自动生成的 `docs/API_INDEX.md`、`docs/api/index.md`、`docs/api/viewer.md`，Python/JS 源码和依赖声明没有冲突。
- 按用户要求，以上三个索引最终原样采用主分支版本，记录于 `d11fb2c`；未再用生成器覆盖它们。assembly 的源码、测试和专门文档仍然保留。
- 这些是本地 Git 提交，尚未 push。`examples/assembly/output/` 内的生成报告、截图和日志仍按原规则忽略，留在本地，没有清理。

## 示例变化

`directions_fibonacci.py`、`directions_socp.py` 和 `stability_demo.py` 现在默认绘制接触区域，并使用新版 `base.ui.add_panel` 增加“接触面显示/隐藏”和“零件不透明度”控制。

共享绘图逻辑在 `_contact_display.py`，主示例只负责传入接触结果。区域直接使用实际 `ContactPatch.regions` 的单元，不以凸包填补孔洞；点/线接触分别用点/线显示。每个面的正反两侧分别建立渲染网格，使背面剔除时仍可从两侧观察。共享单元边在边界线中抵消，避免把每条三角形边都当作接触边界。

颜色保留已有约定：绿 `active`、橙 `near`、红 `interference`、紫 `unknown`。轴孔方向示例继续使用声明的理想零间隙约束，其侧壁与盲孔底面以蓝色独立显示，面板明确标注来源；没有将其伪装为 SDF 检测出的 active 面。通孔不画接触端盖，盲孔只增加底部配合面。

控件仅改变绘图对象，不改变零件的分析位姿、接触分类或力学模型。已有 M1 接触 HTML/WRS 示例继续使用原有绘图入口。

## 验证

- 指定解释器运行 assembly 回归，100 项通过，14.072 s。
- 平面、三面角、平行槽、通孔轴、盲孔轴和六面锁死的方向场景均完成不启动 viewer 的建图检查。
- WRS WebGPU 实际核对两个三面角方向例子、静力堆叠和盲孔侧壁/底面。
- 接触面开关与透明度通过浏览器事件回传 Python；刷新后确认服务端保存了显示状态与 0.4 不透明度，再恢复默认 0.2。
- 新版 `websockets>=14` 依赖与 assembly 的可选依赖均保留。

运行方式不变，在 WRS2 根目录执行：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/directions_fibonacci.py
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/directions_socp.py
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stability_demo.py
```

端口依次为 8891、8892、8893；方向例子加 `--case shaft` 或 `--case blind_shaft` 查看轴孔。`--headless` 仍只计算，不启动 UI。
