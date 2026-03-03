#ifndef LOGIN_DIALOG_H
#define LOGIN_DIALOG_H

#include <QDialog>
#include <QString>

QT_BEGIN_NAMESPACE
namespace Ui { class LoginDialog; }
QT_END_NAMESPACE

class LoginDialog : public QDialog
{
    Q_OBJECT

public:
    enum class UserRole {
        Executive,    // 총책임자
        ControlRoom   // 관제실
    };

public:
    explicit LoginDialog(QWidget *parent = nullptr);
    ~LoginDialog();

    QString userId() const;
    UserRole role() const;

private slots:
    void on_btnLogin_clicked();
    void on_btnExit_clicked();

private:
    bool validateLocal(const QString& id, const QString& pw, UserRole& outRole) const;
    void setStatus(const QString& msg, bool isError = false);

private:
    Ui::LoginDialog *ui;
    QString m_userId;
    UserRole m_role = UserRole::ControlRoom;
};

#endif // LOGIN_DIALOG_H
