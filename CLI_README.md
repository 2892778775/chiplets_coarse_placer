# CLI 自动化使用指南

## 快速开始

```bash
# Expert 算法（默认，推荐）
python run_cli.py --dbx CoWoS_S/CoWoS-S.3dbx --connection CoWoS_S/D2D.connection --output output/

# SA 算法
python run_cli.py --dbx CoWoS_L/CoWoS-L.3dbx --connection CoWoS_L/D2D.connection --output out_l/ --algorithm SA

# 跳过 D2D refinement
python run_cli.py --dbx design.3dbx --connection D2D.connection --output output/ --skip-d2d
```

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--dbx` | 必需 | - | 输入 `.3dbx` 文件路径（等价别名 `--3dbx`） |
| `--connection` | 可选 | "" | D2D connection 文件路径 (`.connection`) |
| `--pi` | 可选 | 自动探测 | PI 隶属关系文件（`LSI.PI`，每行一对 `isolated实例,dominant实例`）；缺省自动读取 `.3dbx` 同目录下的 `LSI.PI` |
| `--output` | 可选 | `output` | 输出目录 |
| `--algorithm` | 可选 | `Expert` | Placer 算法: `SA` / `Expert`（等价别名 `--placer`，大小写不敏感） |
| `--sa-iterations` | 可选 | `5000` | SA 迭代次数（仅 SA 算法生效） |
| `--enclosure` | 可选 | `500.0` | Interposer 最小包围边距 (um) |
| `--skip-d2d` | 可选 | False | 跳过 D2D PHY 对齐优化 |
| `--no-images` | 可选 | False | 跳过 PNG 图像生成 |
| `--no-json` | 可选 | False | 跳过 `score.json` 输出 |
| `--no-csv` | 可选 | False | 跳过 `score.csv` 输出 |
| `--seed` | 可选 | None | 随机种子（SA 算法可复现性） |
| `--dpi` | 可选 | `150` | 图像分辨率 DPI |
| `--quiet` | 可选 | False | 静默模式（仅输出关键信息） |

## 输出文件

运行后在 `--output` 目录下生成：

| 文件 | 说明 |
|------|------|
| `*_export.3dbx` | 顶层 3Dblox 设计文件 |
| `*_export.3dbv` | 包含所有 chiplet 引用的 3Dblox 定义文件 |
| `<chiplet>.3dbv` | 各 chiplet 定义文件 |
| `<chiplet>.3dbo` | 各 chiplet 对象定义文件 |
| `<chiplet>.omap` | 各 chiplet IP 映射文件 |
| `floorplan.png` | 2D 布局可视化图（含 chiplet、D2D 连线、IP 位置） |
| `score_table.png` | 评分表图像（硬约束 + 软约束得分） |
| `score.json` | 机器可读评分报告（JSON） |
| `score.csv` | 表格评分报告（CSV） |

## 算法选择建议

- **Expert**: 基于 D2D 连接规则的专家系统，速度快、结果稳定，适合有明确 D2D 拓扑的设计（如 CoWoS_S/CoWoS_L）。
- **SA**: 模拟退火优化，支持更大搜索空间，适合无 D2D 连接或需要全局优化的场景。需要指定 `--sa-iterations` 控制优化时间。

## 注意事项

1. D2D refinement 步骤在 Expert 算法下有时会破坏已有对齐。CLI 已内置自动回退机制：如果 refinement 导致 hard constraint 违反，则自动回退到 refinement 前状态。
2. 图像生成需要 `matplotlib`（已包含在项目环境中）。
3. 如果连接文件未指定，则视为无 D2D 连接，仅进行 generic placement。
