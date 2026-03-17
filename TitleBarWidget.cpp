#include "TitleBarWidget.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

TitleBarWidget::TitleBarWidget(QWidget* parent)
    : QWidget(parent)
{
    setObjectName("TitleBarWidget");
    setAttribute(Qt::WA_StyledBackground, true);
    setFixedHeight(38);

    m_title = new QLabel(this);
    m_title->setObjectName("titleText");
    m_title->setText(" ");

    m_btnMin = new QPushButton("-", this);
    m_btnMin->setObjectName("btnMin");
    m_btnMin->setFixedSize(42, 28);
    m_btnMin->setFocusPolicy(Qt::NoFocus);

    m_btnMax = new QPushButton("[]", this);
    m_btnMax->setObjectName("btnMax");
    m_btnMax->setFixedSize(42, 28);
    m_btnMax->setFocusPolicy(Qt::NoFocus);

    m_btnClose = new QPushButton("X", this);
    m_btnClose->setObjectName("btnClose");
    m_btnClose->setFixedSize(42, 28);
    m_btnClose->setFocusPolicy(Qt::NoFocus);

    QFont btnFont = font();
    btnFont.setPointSize(11);
    btnFont.setBold(true);
    m_btnMin->setFont(btnFont);
    m_btnMax->setFont(btnFont);
    m_btnClose->setFont(btnFont);

    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(12, 0, 8, 0);
    layout->setSpacing(8);
    layout->addWidget(m_title, 1);
    layout->addWidget(m_btnMin);
    layout->addWidget(m_btnMax);
    layout->addWidget(m_btnClose);
    setLayout(layout);

    setStyleSheet(
        "#TitleBarWidget { background-color: #111522; border-bottom: 1px solid #242c43; }"
        "#titleText { background-color: transparent; color: #eef2ff; font-weight: 700; font-size: 14px; }"
        "QPushButton { background: transparent; color: #b9c4e4; border: none; border-radius: 8px; font-size: 12px; }"
        "QPushButton:hover { background: #232c43; color: #ffffff; }"
        "QPushButton#btnClose:hover { background: #d14f6e; color: #ffffff; }"
    );

    connect(m_btnMin, &QPushButton::clicked, this, &TitleBarWidget::minimizeRequested);
    connect(m_btnMax, &QPushButton::clicked, this, &TitleBarWidget::onMaximizeClicked);
    connect(m_btnClose, &QPushButton::clicked, this, &TitleBarWidget::closeRequested);
}

void TitleBarWidget::setTitleText(const QString& text)
{
    if (m_title) m_title->setText(text);
}

void TitleBarWidget::setMaximizeVisible(bool visible)
{
    if (m_btnMax) m_btnMax->setVisible(visible);
}

void TitleBarWidget::setMinimizeVisible(bool visible)
{
    if (m_btnMin) m_btnMin->setVisible(visible);
}

void TitleBarWidget::setMaximized(bool maximized)
{
    m_isMaximized = maximized;
    if (m_btnMax) m_btnMax->setText(m_isMaximized ? "O" : "[]");
}

void TitleBarWidget::onMaximizeClicked()
{
    if (m_isMaximized) {
        emit restoreRequested();
    } else {
        emit maximizeRequested();
    }
}
