#include <frc/controller/ElevatorFeedforward.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

class Lift : public frc2::SubsystemBase
{
public:
    Lift();

    void driveDirect(float speed);

    frc2::CommandPtr homeCmd();
    frc2::CommandPtr moveCmd(float speed);
    frc2::CommandPtr moveToPosCmd(float position);
    frc2::CommandPtr stopCmd();

private:
    rev::spark::SparkMax m_leftWinch, m_rightWinch;
};