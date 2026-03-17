#include "login_dialog.h"
#include "ui_login_dialog.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QGraphicsDropShadowEffect>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QSize>
#include "TitleBarWidget.h"
#include "FramelessHelper.h"

LoginDialog::LoginDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::LoginDialog)
{
    ui->setupUi(this);
    setupCustomTitleBar();

    QString backgroundPath;
    const QStringList candidates = {
        QDir::cleanPath(QCoreApplication::applicationDirPath() + "/image/ui_background.jpg"),
        QDir::cleanPath(QCoreApplication::applicationDirPath() + "/../image/ui_background.jpg"),
        QDir::cleanPath(QDir::currentPath() + "/image/ui_background.jpg"),
        QStringLiteral("C:/Users/1-14/real_final/main_window/image/ui_background.jpg")
    };
    for (const QString& candidate : candidates) {
        if (QFileInfo::exists(candidate)) {
            backgroundPath = QDir::toNativeSeparators(candidate).replace("\\", "/");
            break;
        }
    }
    if (backgroundPath.isEmpty()) {
        backgroundPath = QStringLiteral("C:/Users/1-14/real_final/main_window/image/ui2.png");
    }

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

    ui->editId->setFocus();

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
        return;
    }

    setStatus("Checking...");

    UserRole role;
    if (!validateLocal(id, pw, role)) {
        setStatus("Login failed", true);
        QMessageBox::warning(this, "Login", "Invalid ID or password.");
        return;
    }

    // ?깃났
    m_userId = id;
    m_role = role;

    setStatus("Login success");
    accept();
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




