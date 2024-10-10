# FTC 17240 : Robot Configuration

## Bot A - 17240-A-RC
REV Robotics Chassis Kit
### Motors
**Control Hub**
|Port|Type|Name|Descriptor|
|----|----|----|----|
|0|REV 40:1 HD Hex Motor|`rightFrontDrive`|Encoder : `frontEncoder`|
|1|REV 40:1 HD Hex Motor|`rightRearDrive`|Encoder : `rightEncoder`|
|2|REV 40:1 HD Hex Motor|`leftRearDrive`|Encoder : `leftEncoder`|
|3|REV 40:1 HD Hex Motor|`leftFrontDrive`||

**Expansion Hub**
|Port|Type|Name|Descriptor|
|----|----|----|----|
|0|REV 40:1 HD Hex Motor|`LeftLiftMotor`||
|1|REV 40:1 HD Hex Motor|`RightLiftMotor`||
|2|Core Hex Motor|`LiftTiltMotor`|Encoder : `LiftTiltEncoder`|

### Servos
**Control Hub**
|Port|Type|Name|Descriptor|
|----|----|----|----|

**Expansion Hub**
|Port|Type|Name|Descriptor|
|----|----|----|----|
|3|Servo|`ClawExtend`||
|4|Servo|`ClawLevel`||
|5|Servo|`ClawClose`||

## Bot B - 17240-B-RC
GoBilda Chassis V4 Kit
### Motors
**Control Hub**
|Port|Type|Name|Descriptor|
|----|----|----|----|
|0|GoBilda 5202/3/4|`rightFrontDrive`|Wire label A - Encoder E1 : `frontEncoder`|
|1|GoBilda 5202/3/4|`rightRearDrive`|Wire label B - Encoder E3 : `rightEncoder`|
|2|GoBilda 5202/3/4|`leftRearDrive`|Wire label C - Encoder E2 : `leftEncoder`|
|3|GoBilda 5202/3/4|`leftFrontDrive`|Wire label D|

**Expansion Hub**
|Port|Type|Name|Desciptor|
|----|----|----|----|
|3|Rev Robotics 40:1 HD Hex Motor|`claw_arm`||

### Servos
**Expansion Hub**
|Port|Type|Name|Descriptor|
|----|----|----|----|
|0|Servo|`claw_lift`|----|
|1|Servo|`claw_close`|----|
