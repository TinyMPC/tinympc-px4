#!/usr/bin/env python3
"""Render the editable HTML extended abstract as a submission PDF."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from html import escape
from html.parser import HTMLParser
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import (
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


@dataclass
class Node:
    tag: str
    attrs: dict[str, str] = field(default_factory=dict)
    children: list["Node | str"] = field(default_factory=list)


class TreeBuilder(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.root = Node("root")
        self.stack = [self.root]

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        node = Node(tag, {key: value or "" for key, value in attrs})
        self.stack[-1].children.append(node)
        if tag not in {"br", "meta", "link", "img", "hr"}:
            self.stack.append(node)

    def handle_startendtag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self.handle_starttag(tag, attrs)
        if self.stack[-1].tag == tag:
            self.stack.pop()

    def handle_endtag(self, tag: str) -> None:
        for index in range(len(self.stack) - 1, 0, -1):
            if self.stack[index].tag == tag:
                del self.stack[index:]
                return

    def handle_data(self, data: str) -> None:
        self.stack[-1].children.append(data)


def nodes(node: Node, tag: str) -> list[Node]:
    found: list[Node] = []
    for child in node.children:
        if isinstance(child, Node):
            if child.tag == tag:
                found.append(child)
            found.extend(nodes(child, tag))
    return found


def direct_nodes(node: Node, tags: set[str] | None = None) -> list[Node]:
    return [
        child
        for child in node.children
        if isinstance(child, Node) and (tags is None or child.tag in tags)
    ]


def markup(node: Node | str) -> str:
    if isinstance(node, str):
        return escape(node).replace("\n", " ")
    content = "".join(markup(child) for child in node.children)
    if node.tag == "strong":
        return f"<b>{content}</b>"
    if node.tag == "em":
        return f"<i>{content}</i>"
    if node.tag == "code":
        return f'<font name="Courier">{content}</font>'
    if node.tag == "br":
        return "<br/>"
    if node.tag == "a":
        return content
    return content


def text(node: Node) -> str:
    return "".join(
        child if isinstance(child, str) else text(child) for child in node.children
    ).strip()


def table_rows(node: Node, cell_style: ParagraphStyle) -> list[list[Paragraph]]:
    result: list[list[Paragraph]] = []
    for row in nodes(node, "tr"):
        cells = direct_nodes(row, {"th", "td"})
        result.append([Paragraph(markup(cell).strip(), cell_style) for cell in cells])
    return result


def make_styles() -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "title": ParagraphStyle(
            "PaperTitle", parent=base["Title"], fontName="Times-Bold",
            fontSize=17, leading=17.5, alignment=TA_CENTER, spaceAfter=3,
        ),
        "author": ParagraphStyle(
            "Author", parent=base["Normal"], fontName="Times-Bold",
            fontSize=11, leading=11.3, alignment=TA_CENTER, spaceAfter=4,
        ),
        "body": ParagraphStyle(
            "Body", parent=base["BodyText"], fontName="Times-Roman",
            fontSize=11, leading=11.15, alignment=TA_JUSTIFY, spaceAfter=2,
        ),
        "abstract": ParagraphStyle(
            "Abstract", parent=base["BodyText"], fontName="Times-Roman",
            fontSize=11, leading=11.15, alignment=TA_JUSTIFY,
            leftIndent=10, rightIndent=10, spaceAfter=2,
        ),
        "heading": ParagraphStyle(
            "Heading", parent=base["Heading2"], fontName="Times-Bold",
            fontSize=11, leading=11.2, alignment=TA_LEFT,
            spaceBefore=2.5, spaceAfter=1, keepWithNext=True,
        ),
        "caption": ParagraphStyle(
            "Caption", parent=base["BodyText"], fontName="Times-Roman",
            fontSize=11, leading=11.05, alignment=TA_CENTER,
            leftIndent=8, rightIndent=8, spaceAfter=2,
        ),
        "cell": ParagraphStyle(
            "Cell", parent=base["BodyText"], fontName="Times-Roman",
            fontSize=11, leading=11.0, alignment=TA_CENTER,
        ),
        "reference": ParagraphStyle(
            "Reference", parent=base["BodyText"], fontName="Times-Roman",
            fontSize=11, leading=11.0, alignment=TA_LEFT,
            leftIndent=14, firstLineIndent=-14, spaceAfter=1,
        ),
        "bullet": ParagraphStyle(
            "Bullet", parent=base["BodyText"], fontName="Times-Roman",
            fontSize=11, leading=11.1, alignment=TA_JUSTIFY,
            leftIndent=14, firstLineIndent=-8, bulletIndent=2, spaceAfter=1,
        ),
    }


def render(source: Path, output: Path) -> None:
    parser = TreeBuilder()
    parser.feed(source.read_text(encoding="utf-8"))
    body_nodes = nodes(parser.root, "body")
    if len(body_nodes) != 1:
        raise RuntimeError("expected exactly one HTML body")

    styles = make_styles()
    story = []
    sections = direct_nodes(body_nodes[0], {"section"})
    for section in sections:
        for node in direct_nodes(section):
            classes = set(node.attrs.get("class", "").split())
            if node.tag == "h1":
                story.append(Paragraph(markup(node).strip(), styles["title"]))
            elif node.tag == "h2":
                story.append(Paragraph(markup(node).strip(), styles["heading"]))
            elif node.tag == "p":
                if "author" in classes:
                    style = styles["author"]
                elif "abstract" in classes:
                    style = styles["abstract"]
                elif "caption" in classes:
                    style = styles["caption"]
                else:
                    style = styles["body"]
                story.append(Paragraph(markup(node).strip(), style))
            elif node.tag == "table":
                table_class = node.attrs.get("class", "")
                rows = table_rows(node, styles["cell"])
                if "architecture" in table_class:
                    widths = [1.12, 0.22, 1.52, 0.22, 1.66, 0.22, 0.78]
                    table = Table(rows, colWidths=[value * inch for value in widths])
                    commands = [
                        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                        ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                        ("LEFTPADDING", (0, 0), (-1, -1), 2),
                        ("RIGHTPADDING", (0, 0), (-1, -1), 2),
                        ("TOPPADDING", (0, 0), (-1, -1), 3),
                        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
                    ]
                    for column in (0, 2, 4, 6):
                        commands.append(("BOX", (column, 0), (column, -1), 0.7, colors.black))
                else:
                    table = Table(rows, colWidths=[2.48 * inch, 2.00 * inch, 2.42 * inch], repeatRows=1)
                    commands = [
                        ("GRID", (0, 0), (-1, -1), 0.55, colors.black),
                        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e9e9e9")),
                        ("FONTNAME", (0, 0), (-1, 0), "Times-Bold"),
                        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                        ("LEFTPADDING", (0, 0), (-1, -1), 3),
                        ("RIGHTPADDING", (0, 0), (-1, -1), 3),
                        ("TOPPADDING", (0, 0), (-1, -1), 2),
                        ("BOTTOMPADDING", (0, 0), (-1, -1), 2),
                    ]
                table.setStyle(TableStyle(commands))
                story.extend([Spacer(1, 2), table, Spacer(1, 1)])
            elif node.tag == "ul":
                for item in direct_nodes(node, {"li"}):
                    story.append(Paragraph(markup(item).strip(), styles["bullet"], bulletText="•"))
            elif node.tag == "div" and "references" in classes:
                for reference in direct_nodes(node, {"p"}):
                    story.append(Paragraph(markup(reference).strip(), styles["reference"]))

    output.parent.mkdir(parents=True, exist_ok=True)
    document = SimpleDocTemplate(
        str(output), pagesize=letter,
        leftMargin=0.60 * inch, rightMargin=0.60 * inch,
        topMargin=0.40 * inch, bottomMargin=0.40 * inch,
        title="From Solver Prototype to Maintainable Flight-Stack Integration",
        author="Ishaan Mahajan",
        subject="Track 1 extended abstract on sustainable robotics software",
    )
    document.build(story)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", type=Path,
        default=Path(__file__).with_name("sustainable_tinympc_px4_extended_abstract.html"),
    )
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).with_name("sustainable_tinympc_px4_extended_abstract.pdf"),
    )
    args = parser.parse_args()
    render(args.source, args.output)


if __name__ == "__main__":
    main()
