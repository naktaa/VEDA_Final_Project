#include "login_dialog.h"
#include "ui_login_dialog.h"

#include <QMessageBox>

LoginDialog::LoginDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::LoginDialog)
{
    ui->setupUi(this);

    setStyleSheet(R"(
QDialog {
    background-color: #6d6a64;
    color: #333333;
}
QLabel {
    background-color: #6d6a64;
    color: #333333;
}
QLineEdit {
    background-color: #e3ddda;
    color: #333333;
    border: 1px solid #988575;
    border-radius: 2px;
    padding: 5px 8px;
}
QPushButton {
    background-color: #e3ddda;
    color: #333333;
    border: 1px solid #665950;
    border-radius: 2px;
    min-height: 26px;
    padding: 3px 10px;
    font-weight: 600;
}
QPushButton:hover {
    background-color: #bcb2aa;
}
QPushButton:pressed {
    background-color: #b3a79e;
}
QLabel#labelTitle {
    color: #333333;
    font-weight: 700;
}
)");

    setModal(true);

    ui->editId->setFocus();
    ui->labelStatus->setText("Ready");

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

    // ?�공
    m_userId = id;
    m_role = role;

    setStatus("Login success");
    accept();
}

bool LoginDialog::validateLocal(const QString& id, const QString& pw, UserRole& outRole) const
{
    /*
     * ?�모??계정 ?�책
     *
     * 총책?�자:
     *   - hospital_director / 1111
     *   - vice_director     / 2222
     *
     * 관?�실:
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
        ui->labelStatus->setStyleSheet("color: #ffd7a8; font-weight: 600;");
    } else {
        ui->labelStatus->setStyleSheet("color: #f3efe9;");
    }
}






