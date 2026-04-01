#include "login_dialog.h"
#include "ui_login_dialog.h"

#include <QGraphicsDropShadowEffect>
#include <QPropertyAnimation>
#include <QEasingCurve>
#include <QLineEdit>
#include <QSettings>
#include <QVBoxLayout>
#include <QSize>
#include "TitleBarWidget.h"
#include "FramelessHelper.h"

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

LoginDialog::LoginDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::LoginDialog)
{
    ui->setupUi(this);
    setupCustomTitleBar();

    const QString backgroundPath = QStringLiteral(":/image/ui_background.jpg");

    setStyleSheet(QString(R"(
QDialog {
    background-color: #12162a;
    color: #eef2ff;
}
QWidget#widget {
    border-image: url(%1) 0 0 0 0 stretch stretch;
    border: none;
}
QWidget#verticalLayoutWidget {
    background-color: rgba(34, 37, 63, 226);
    border: 1px solid rgba(91, 98, 145, 170);
    border-radius: 18px;
}
QLabel {
    background-color: transparent;
    color: #e8ecff;
}
QLineEdit {
    background-color: rgba(88, 92, 128, 210);
    color: #f7f8ff;
    border: 1px solid rgba(115, 121, 171, 140);
    border-radius: 8px;
    padding: 9px 12px;
    min-height: 22px;
}
QPushButton {
    background-color: rgba(255, 255, 255, 0.92);
    color: #1f2742;
    border: none;
    border-radius: 14px;
    min-height: 28px;
    padding: 5px 16px;
    font-weight: 700;
}
QPushButton:hover {
    background-color: #ffffff;
}
QPushButton:pressed {
    background-color: #e9ecff;
}
QLabel#labelTitle {
    color: #ffffff;
    font-weight: 800;
    font-size: 20px;
}
QLabel#labelStatus {
    color: #9aa4c7;
    font-size: 11px;
}
QLabel#ID, QLabel#Password {
    color: #8d97bb;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
}
)").arg(backgroundPath));

    setModal(true);

    setWindowTitle("Sentinel Fusion Login");

    if (ui->widget) {
        ui->widget->setMinimumSize(980, 620);
        ui->widget->setMaximumSize(980, 620);
    }
    if (ui->verticalLayoutWidget) {
        ui->verticalLayoutWidget->setGeometry(325, 148, 330, 306);
        auto* shadow = new QGraphicsDropShadowEffect(ui->verticalLayoutWidget);
        shadow->setBlurRadius(28);
        shadow->setOffset(0, 12);
        shadow->setColor(QColor(10, 10, 30, 110));
        ui->verticalLayoutWidget->setGraphicsEffect(shadow);
    }
    if (ui->verticalLayout_root) {
        ui->verticalLayout_root->setContentsMargins(24, 20, 24, 18);
        ui->verticalLayout_root->setSpacing(8);
    }
    if (ui->labelTitle) ui->labelTitle->setText("Sentinel Fusion");
    if (ui->ID) ui->ID->setText("USERNAME");
    if (ui->Password) ui->Password->setText("PASSWORD");
    if (ui->editId) ui->editId->setPlaceholderText("hello@samuelmay.co");
    if (ui->editPw) ui->editPw->setPlaceholderText("password");
    if (ui->btnLogin) ui->btnLogin->setText("LOG IN");
    if (ui->labelTitle) {
        ui->labelTitle->setMinimumHeight(42);
        ui->labelTitle->setMaximumHeight(42);
        QFont f = ui->labelTitle->font();
        f.setPointSize(18);
        f.setBold(true);
        ui->labelTitle->setFont(f);
    }
    if (ui->ID) {
        QFont f = ui->ID->font();
        f.setPointSize(9);
        f.setBold(true);
        ui->ID->setFont(f);
    }
    if (ui->Password) {
        QFont f = ui->Password->font();
        f.setPointSize(9);
        f.setBold(true);
        ui->Password->setFont(f);
    }
    if (ui->editId) {
        ui->editId->setMinimumWidth(282);
        ui->editId->setMaximumWidth(282);
        ui->editId->setMinimumHeight(40);
        ui->editId->setMaximumHeight(40);
    }
    if (ui->editPw) {
        ui->editPw->setMinimumWidth(282);
        ui->editPw->setMaximumWidth(282);
        ui->editPw->setMinimumHeight(40);
        ui->editPw->setMaximumHeight(40);
        connect(ui->editPw, &QLineEdit::textChanged, this, [this](const QString&) {
            updateCapsLockWarning();
        });
    }
    if (ui->btnLogin) {
        ui->btnLogin->setMinimumWidth(172);
        ui->btnLogin->setMaximumWidth(172);
        ui->btnLogin->setMinimumHeight(38);
        ui->btnLogin->setMaximumHeight(38);
    }
    if (ui->horizontalLayout) {
        ui->horizontalLayout->setContentsMargins(0, 10, 0, 0);
        ui->horizontalLayout->setSpacing(0);
        ui->horizontalLayout->setAlignment(Qt::AlignHCenter);
    }
    if (ui->labelStatus) {
        ui->labelStatus->setText("FORGOT YOUR PASSWORD?");
        ui->labelStatus->setAlignment(Qt::AlignCenter);
        ui->labelStatus->setMaximumHeight(18);
        ui->labelStatus->setStyleSheet("color: rgba(154, 164, 199, 0.55); background: transparent; font-size: 10px; font-weight: 700;");
    }

    QSettings settings("VEDA", "SentinelFusion");
    if (ui->editId) {
        ui->editId->setText(settings.value("login/last_id").toString());
    }

    ui->editId->setFocus();
    updateCapsLockWarning();

    ui->btnLogin->setDefault(true);
}

LoginDialog::~LoginDialog()
{
    delete ui;
}

QString LoginDialog::userId() const
{
    return m_userId;
}

LoginDialog::UserRole LoginDialog::role() const
{
    return m_role;
}

void LoginDialog::on_btnExit_clicked()
{
    reject();
}

void LoginDialog::on_btnLogin_clicked()
{
    const QString id = ui->editId->text().trimmed();
    const QString pw = ui->editPw->text();

    if (id.isEmpty() || pw.isEmpty()) {
        setStatus("ID/PW required", true);
        shakeLoginCard();
        return;
    }

    setStatus("Checking...");

    UserRole role;
    if (!validateLocal(id, pw, role)) {
        setStatus("Login failed", true);
        shakeLoginCard();
        return;
    }

    // ?깃났
    m_userId = id;
    m_role = role;
    QSettings settings("VEDA", "SentinelFusion");
    settings.setValue("login/last_id", id);

    setStatus("Login success");
    startLoginSuccessAnimation();
}

bool LoginDialog::validateLocal(const QString& id, const QString& pw, UserRole& outRole) const
{
    /*
     * ?곕え??怨꾩젙 ?뺤콉
     *
     * 珥앹콉?꾩옄:
     *   - hospital_director / 1111
     *   - vice_director     / 2222
     *
     * 愿?쒖떎:
     *   - control / 0000
     */

    if (id == "hospital_director" && pw == "1111") {
        outRole = UserRole::Executive;
        return true;
    }

    if (id == "vice_director" && pw == "2222") {
        outRole = UserRole::Executive;
        return true;
    }

    if (id == "control" && pw == "0000") {
        outRole = UserRole::ControlRoom;
        return true;
    }

    return false;
}

void LoginDialog::setStatus(const QString& msg, bool isError)
{
    ui->labelStatus->setText(msg);

    if (isError) {
        ui->labelStatus->setStyleSheet("color: #ffb4b4; font-weight: 700; background: transparent; font-size: 10px;");
    } else {
        ui->labelStatus->setStyleSheet("color: rgba(154, 164, 199, 0.55); background: transparent; font-size: 10px; font-weight: 700;");
    }
}







void LoginDialog::shakeLoginCard()
{
    if (!ui || !ui->verticalLayoutWidget) return;

    auto* card = ui->verticalLayoutWidget;
    const QRect base = card->geometry();

    auto* anim = new QPropertyAnimation(card, "geometry", card);
    anim->setDuration(280);
    anim->setEasingCurve(QEasingCurve::OutCubic);
    anim->setKeyValueAt(0.0, base);
    anim->setKeyValueAt(0.10, QRect(base.x() - 14, base.y(), base.width(), base.height()));
    anim->setKeyValueAt(0.25, QRect(base.x() + 12, base.y(), base.width(), base.height()));
    anim->setKeyValueAt(0.45, QRect(base.x() - 10, base.y(), base.width(), base.height()));
    anim->setKeyValueAt(0.65, QRect(base.x() + 8, base.y(), base.width(), base.height()));
    anim->setKeyValueAt(0.82, QRect(base.x() - 4, base.y(), base.width(), base.height()));
    anim->setKeyValueAt(1.0, base);
    anim->start(QAbstractAnimation::DeleteWhenStopped);
}

void LoginDialog::updateCapsLockWarning()
{
    if (!ui || !ui->labelStatus || !ui->editPw) return;
#ifdef Q_OS_WIN
    const bool capsOn = (GetKeyState(VK_CAPITAL) & 0x0001) != 0;
#else
    const bool capsOn = false;
#endif
    if (capsOn && ui->editPw->hasFocus()) {
        ui->labelStatus->setText("CAPS LOCK is ON");
        ui->labelStatus->setStyleSheet("color: #f3c86b; font-weight: 700; background: transparent; font-size: 10px;");
    }
}

void LoginDialog::startLoginSuccessAnimation()
{
    auto* anim = new QPropertyAnimation(this, "windowOpacity", this);
    anim->setDuration(180);
    anim->setStartValue(1.0);
    anim->setEndValue(0.0);
    connect(anim, &QPropertyAnimation::finished, this, [this]() {
        setWindowOpacity(1.0);
        accept();
    });
    anim->start(QAbstractAnimation::DeleteWhenStopped);
}
void LoginDialog::setupCustomTitleBar()
{
    setWindowFlags(windowFlags() | Qt::FramelessWindowHint);

    m_titleBar = new TitleBarWidget(this);
    m_titleBar->setTitleText(windowTitle());
    m_titleBar->setMinimizeVisible(false);
    m_titleBar->setMaximizeVisible(false);

    connect(this, &QWidget::windowTitleChanged,
            m_titleBar, &TitleBarWidget::setTitleText);
    connect(m_titleBar, &TitleBarWidget::closeRequested,
            this, &QWidget::close);

    QVBoxLayout* root = qobject_cast<QVBoxLayout*>(layout());
    if (!root) {
        root = new QVBoxLayout(this);
        root->setContentsMargins(0, 0, 0, 0);
        root->setSpacing(0);
        setLayout(root);
    }

    if (ui->widget)
        ui->widget->setParent(this);

    root->addWidget(m_titleBar);
    if (ui->widget)
        root->addWidget(ui->widget, 1);

    root->setStretch(0, 0);
    root->setStretch(1, 1);

    if (ui->widget) {
        const QSize contentSize(980, 620);
        const int titleH = m_titleBar->height();
        resize(contentSize.width(), contentSize.height() + titleH);
        setFixedSize(size());
    }
}

bool LoginDialog::nativeEvent(const QByteArray& eventType, void* message, qintptr* result)
{
    if (FramelessHelper::handleNativeEvent(this, m_titleBar, eventType, message, result))
        return true;
    return QDialog::nativeEvent(eventType, message, result);
}

void LoginDialog::changeEvent(QEvent* event)
{
    if (event && event->type() == QEvent::WindowStateChange) {
        if (m_titleBar) m_titleBar->setMaximized(isMaximized());
    }
    QDialog::changeEvent(event);
}




