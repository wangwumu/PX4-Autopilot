#!/usr/bin/env python3
"""
将 offboard_obstacle_avoidance_uxrce.md 转换为 PDF。
要求：
- 表格格式正确
- 表格不跨页 (page-break-inside: avoid)
- 代码块有语法高亮样式
- 中文字体支持
"""

import markdown
from weasyprint import HTML
import os
import sys

# ── CSS 样式 ────────────────────────────────────────────────────

CSS = """
@page {
  size: A4;
  margin: 2cm 2.2cm 2cm 2.2cm;
  @bottom-center {
    content: counter(page);
    font-size: 9pt;
    color: #888;
    font-family: 'SimHei', 'DejaVu Sans', sans-serif;
  }
}

body {
  font-family: 'SimHei', 'STSong', 'DejaVu Sans', sans-serif;
  font-size: 10.5pt;
  line-height: 1.65;
  color: #24292e;
}

/* ── 标题层级 ── */
h1 {
  font-size: 20pt;
  border-bottom: 2.5px solid #1a73e8;
  padding-bottom: 6px;
  margin-top: 28px;
  margin-bottom: 14px;
  color: #1a3c6d;
}

h2 {
  font-size: 15pt;
  border-bottom: 1.2px solid #c0c0c0;
  padding-bottom: 4px;
  margin-top: 24px;
  margin-bottom: 10px;
  color: #2c5f2d;
}

h3 {
  font-size: 12.5pt;
  margin-top: 18px;
  margin-bottom: 8px;
  color: #3e4e5e;
}

h4 {
  font-size: 11pt;
  margin-top: 14px;
  margin-bottom: 6px;
  color: #555;
}

/* ── 段落和行内 ── */
p { margin: 6px 0; }

strong { color: #1a3c6d; }

a { color: #1a73e8; text-decoration: none; }

hr {
  border: none;
  border-top: 1px solid #ddd;
  margin: 18px 0;
}

/* ── 行内代码 ── */
code {
  font-family: 'DejaVu Sans Mono', 'SimHei', monospace;
  font-size: 9.2pt;
  background: #f0f0f0;
  padding: 1px 5px;
  border-radius: 3px;
  color: #c7254e;
}

/* ── 代码块 ── */
pre {
  font-family: 'DejaVu Sans Mono', 'SimHei', monospace;
  font-size: 9pt;
  background: #f6f8fa;
  border: 1px solid #d1d5db;
  border-left: 4px solid #1a73e8;
  padding: 10px 14px;
  line-height: 1.45;
  white-space: pre-wrap;
  word-wrap: break-word;
  border-radius: 4px;
  margin: 10px 0;
  page-break-inside: avoid;
}

pre code {
  background: transparent;
  padding: 0;
  color: #24292e;
  font-size: 9pt;
}

/* ── 内联 HTML 中的 ASCII 图表 ── */
pre:has(code:empty) {
  background: #fafbfc;
  border-left: 4px solid #6a737d;
}

/* ── 表格 ── */
table {
  width: 100%;
  border-collapse: collapse;
  margin: 10px 0 12px 0;
  font-size: 9.5pt;
  /* ★ 关键：表格不跨页 */
  page-break-inside: avoid;
}

thead {
  display: table-header-group;
}

tr {
  page-break-inside: avoid;
}

th {
  background: #1a73e8;
  color: #ffffff;
  font-weight: 600;
  padding: 7px 10px;
  text-align: left;
  border: 1px solid #1558b0;
}

td {
  padding: 5px 10px;
  border: 1px solid #d1d5db;
  vertical-align: top;
}

tr:nth-child(even) td {
  background: #f8f9fa;
}

/* 表格内的代码 */
td code {
  font-size: 8.5pt;
  white-space: nowrap;
}

/* ── 列表 ── */
ul, ol {
  padding-left: 24px;
  margin: 6px 0;
}

li {
  margin: 2px 0;
}

/* ── 引用块 ── */
blockquote {
  border-left: 4px solid #ffc107;
  background: #fffde7;
  margin: 10px 0;
  padding: 8px 14px;
  color: #5d4037;
  page-break-inside: avoid;
}

blockquote p {
  margin: 4px 0;
}

/* ── Diagrams (HTML 图表) ── */

/* 通用：图表容器不跨页 */
.diagram {
  page-break-inside: avoid;
  margin: 12px 0;
}

/* 架构图：两列布局 */
.arch-row {
  display: flex;
  gap: 0;
  align-items: stretch;
  margin: 0;
}
.arch-col {
  flex: 1;
  border: 2px solid #1a73e8;
  border-radius: 6px;
  padding: 10px 14px;
  background: #f8faff;
  font-size: 9.5pt;
  line-height: 1.5;
}
.arch-col-title {
  font-size: 11pt;
  font-weight: 700;
  color: #1a73e8;
  border-bottom: 1px solid #c8daf5;
  padding-bottom: 6px;
  margin-bottom: 8px;
  text-align: center;
}
.arch-col ul { margin: 3px 0; padding-left: 18px; }
.arch-col li { margin: 2px 0; }
.arch-col .sub { color: #555; font-size: 9pt; margin-left: 4px; }
.arch-col .section-label {
  font-weight: 600;
  color: #2c5f2d;
  border-top: 1px dashed #c0c0c0;
  padding-top: 6px;
  margin-top: 8px;
}
.arch-arrow {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  min-width: 100px;
  padding: 0 8px;
  text-align: center;
  font-size: 9pt;
  color: #666;
}
.arch-arrow .proto {
  font-weight: 700;
  color: #c7254e;
  font-size: 9.5pt;
}
.arch-arrow .arrow-sym {
  font-size: 14pt;
  color: #1a73e8;
  padding: 4px 0;
}

/* 步骤流程图 */
.step-box {
  border: 1.5px solid #d1d5db;
  border-radius: 6px;
  margin: 10px 0;
  overflow: hidden;
}
.step-header {
  background: #e8f0fe;
  padding: 7px 14px;
  font-weight: 700;
  font-size: 10.5pt;
  color: #1a3c6d;
  border-bottom: 1px solid #c8daf5;
}
.step-body {
  padding: 8px 14px;
  font-size: 9.5pt;
  line-height: 1.55;
}
.step-body ul { margin: 4px 0; padding-left: 20px; }
.step-body li { margin: 2px 0; }

/* 消息时序图 */
.msg-timeline {
  font-size: 9.5pt;
  line-height: 1.5;
  border-collapse: collapse;
  width: 100%;
  page-break-inside: avoid;
}
.msg-timeline td {
  padding: 3px 8px;
  border: none;
  vertical-align: middle;
}
.msg-timeline .ts-col {
  width: 60px;
  text-align: right;
  color: #888;
  font-size: 9pt;
  white-space: nowrap;
}
.msg-timeline .from-col {
  width: 42%;
  text-align: right;
  padding-right: 4px;
  font-family: 'DejaVu Sans Mono', 'SimHei', monospace;
  font-size: 9pt;
}
.msg-timeline .to-col {
  width: 42%;
  text-align: left;
  padding-left: 4px;
  font-family: 'DejaVu Sans Mono', 'SimHei', monospace;
  font-size: 9pt;
}
.msg-timeline .arr-col {
  width: 40px;
  text-align: center;
  font-size: 10pt;
  color: #1a73e8;
}
.msg-timeline .note-col {
  width: auto;
  font-size: 8.5pt;
  color: #666;
}
.msg-timeline .px4-note {
  text-align: left;
}
.msg-timeline .cc-note {
  text-align: right;
}
.msg-timeline tr.phase-row td {
  padding-top: 8px;
  color: #2c5f2d;
  font-weight: 600;
  font-size: 9pt;
}
.msg-timeline tr.sep-row td {
  border-top: 1px dashed #ddd;
}

/* 状态机图 */
.state-machine {
  font-size: 10pt;
  line-height: 1.6;
}
.state-machine .state {
  display: inline-block;
  border: 2px solid #1a73e8;
  border-radius: 12px;
  padding: 5px 14px;
  margin: 3px;
  font-weight: 600;
  background: #f0f5ff;
  text-align: center;
  min-width: 80px;
}
.state-machine .state.active {
  background: #1a73e8;
  color: #fff;
}
.state-machine .state.warn {
  border-color: #e67e22;
  background: #fff8f0;
}
.state-machine .state.err {
  border-color: #c0392b;
  background: #fff0f0;
}
.state-machine .arrow {
  display: inline-block;
  margin: 0 6px;
  color: #888;
  font-size: 11pt;
}
.state-machine .label {
  display: inline-block;
  font-size: 8.5pt;
  color: #666;
  margin: 0 4px;
  vertical-align: middle;
}
.state-row {
  text-align: center;
  margin: 8px 0;
}

/* 心跳层级图 */
.heartbeat-layer {
  display: flex;
  flex-direction: column;
  gap: 8px;
  margin: 10px 0;
}
.hb-layer {
  border: 1.5px solid #d1d5db;
  border-radius: 6px;
  padding: 8px 14px;
  page-break-inside: avoid;
}
.hb-layer.l1 { border-left: 5px solid #3498db; background: #f0f8ff; }
.hb-layer.l2 { border-left: 5px solid #e74c3c; background: #fff5f5; }
.hb-layer.l3 { border-left: 5px solid #27ae60; background: #f5fff8; }
.hb-layer .hb-title {
  font-weight: 700;
  font-size: 10pt;
  margin-bottom: 4px;
}
.hb-layer.l1 .hb-title { color: #2980b9; }
.hb-layer.l2 .hb-title { color: #c0392b; }
.hb-layer.l3 .hb-title { color: #1e8449; }
.hb-layer .hb-detail {
  font-size: 9pt;
  color: #555;
  line-height: 1.5;
}

/* 控制类型对照表 */
.ctrl-type-list {
  font-size: 9.5pt;
  line-height: 1.6;
}
.ctrl-type-list .ctrl-item {
  padding: 3px 0;
  padding-left: 12px;
}
.ctrl-type-list .ctrl-name {
  font-weight: 600;
  font-family: 'DejaVu Sans Mono', 'SimHei', monospace;
  font-size: 9pt;
  color: #1a3c6d;
}

/* 避障步骤 */
.avoid-step {
  border: 1px solid #e0e0e0;
  border-radius: 5px;
  margin: 6px 0;
  overflow: hidden;
  page-break-inside: avoid;
}
.avoid-step .step-num {
  background: #1a73e8;
  color: #fff;
  font-weight: 700;
  font-size: 9pt;
  padding: 4px 10px;
  display: inline-block;
  min-width: 60px;
}
.avoid-step .step-desc {
  padding: 6px 10px;
  font-size: 9.5pt;
}
.avoid-step .step-desc ul {
  margin: 2px 0;
  padding-left: 18px;
}

/* 关键原则框 */
.principle-box {
  border: 2px solid #e74c3c;
  border-radius: 6px;
  background: #fffafa;
  padding: 10px 14px;
  margin: 10px 0;
  page-break-inside: avoid;
}
.principle-box .principle-num {
  font-weight: 700;
  color: #c0392b;
  font-size: 10.5pt;
}
.principle-box .principle-title {
  font-weight: 700;
  color: #333;
  font-size: 10pt;
}
.principle-box .principle-body {
  margin-top: 4px;
  font-size: 9.5pt;
  color: #555;
  line-height: 1.5;
}

/* 重要提示框 ── */
/* 为包含 ★ 或 ⚠ 的段落添加样式 */
p:has(strong) {
  /* 不做特殊处理，仅保留文字样式 */
}

/* ── 有序/无序列表的嵌套 ── */
ul ul, ol ol, ul ol, ol ul {
  margin: 2px 0;
}

/* ── 第一页不要页码 ── */
@page :first {
  @bottom-center {
    content: none;
  }
}
"""

# ── HTML 模板 ──────────────────────────────────────────────────

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>ROS 2 Offboard 避障控制完整指南</title>
  <style>{css}</style>
</head>
<body>
{body}
</body>
</html>"""

# ── 主流程 ─────────────────────────────────────────────────────

def main():
    md_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'offboard_obstacle_avoidance_uxrce.md'
    )
    pdf_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'offboard_obstacle_avoidance_uxrce.pdf'
    )

    if not os.path.exists(md_path):
        print(f"错误: 找不到 {md_path}")
        sys.exit(1)

    print(f"读取: {md_path}")

    with open(md_path, 'r', encoding='utf-8') as f:
        md_content = f.read()

    # 确保所有 `` 围栏代码块被正确处理
    # Python-markdown 的 fenced_code 扩展处理 ``` 围栏

    print("转换 Markdown → HTML ...")

    md = markdown.Markdown(extensions=[
        'fenced_code',       # ``` 围栏代码块
        'tables',            # GFM 表格
        'codehilite',        # 代码高亮 (需要 Pygments)
        'toc',               # 目录
        'nl2br',             # 换行转 <br>
        'sane_lists',        # 更好的列表处理
    ])

    html_body = md.convert(md_content)

    # 组合完整 HTML
    full_html = HTML_TEMPLATE.format(css=CSS, body=html_body)

    # 写入临时 HTML (方便调试)
    html_tmp = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '.offboard_avoidance_tmp.html'
    )
    with open(html_tmp, 'w', encoding='utf-8') as f:
        f.write(full_html)

    print(f"临时 HTML: {html_tmp}")
    print("渲染 PDF (使用 WeasyPrint) ...")

    # 生成 PDF
    HTML(string=full_html).write_pdf(pdf_path)

    print(f"✅ PDF 已生成: {pdf_path}")
    print(f"   文件大小: {os.path.getsize(pdf_path) / 1024:.1f} KB")


if __name__ == '__main__':
    main()
