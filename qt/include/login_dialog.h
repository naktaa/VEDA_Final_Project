#ifndef LOGIN_DIALOG_H
#define LOGIN_DIALOG_H

#include <QDialog>
#include <QString>

QT_BEGIN_NAMESPACE
namespace Ui { class LoginDialog; }
QT_END_NAMESPACE

class TitleBarWidget;
class QEvent;

class LoginDialog : public QDialog
{
    Q_OBJECT

public:
    enum class UserRole {
        Executive,    // 총책?�자
        ControlRoom   // 관?�실
    };

public:
    explicit LoginDialog(QWidget *parent = nullptr);
    ~LoginDialog();

    QString userId() const;
    UserRole role() const;

protected:
    bool nativeEvent(const QByteArray& eventType, void* message, qintptr* result) override;
    void changeEvent(QEvent* event) override;

private slots:
    void on_btnLogin_clicked();
    void on_btnExit_clicked();

private:
    void setupCustomTitleBar();
    bool validateLocal(const QString& id, const QString& pw, UserRole& outRole) const;
    void setStatus(const QString& msg, bool isError = false);
    void shakeLoginCard();
    void updateCapsLockWarning();
    void startLoginSuccessAnimation();

private:
    Ui::LoginDialog *ui;
    TitleBarWidget* m_titleBar = nullptr;
    QString m_userId;
    UserRole m_role = UserRole::ControlRoom;
};

#endif // LOGIN_DIALOG_H





