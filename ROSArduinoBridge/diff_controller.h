/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:

   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct
{
  double TargetTicksPerFrame; // target speed in ticks per frame
  long Encoder;               // encoder count
  long PrevEnc;               // last encoder count

  /*
   * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   */
  int PrevInput; // last input
  // int PrevErr;                   // last error

  /*
   * Using integrated term (ITerm) instead of integrated error (Ierror),
   * to allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // int Ierror;
  int ITerm; // integrated term

  long output; // last motor setting
} SetPointInfo;

SetPointInfo leftFrontPID, rightFrontPID, leftRearPID, rightRearPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 * In particular, assign both Encoder and PrevEnc the current encoder value
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * Note that the assumption here is that PID is only turned on
 * when going from stop to moving, that's why we can init everything on zero.
 */
void resetPID()
{
  leftFrontPID.TargetTicksPerFrame = 0.0;
  leftFrontPID.Encoder = readEncoder(setMotorSpeeds);
  leftFrontPID.PrevEnc = leftFrontPID.Encoder;
  leftFrontPID.output = 0;
  leftFrontPID.PrevInput = 0;
  leftFrontPID.ITerm = 0;

  rightFrontPID.TargetTicksPerFrame = 0.0;
  rightFrontPID.Encoder = readEncoder(RIGHT_FRONT);
  rightFrontPID.PrevEnc = rightFrontPID.Encoder;
  rightFrontPID.output = 0;
  rightFrontPID.PrevInput = 0;
  rightFrontPID.ITerm = 0;

  leftRearPID.TargetTicksPerFrame = 0.0;
  leftRearPID.Encoder = readEncoder(LEFT_REAR);
  leftRearPID.PrevEnc = leftRearPID.Encoder;
  leftRearPID.output = 0;
  leftRearPID.PrevInput = 0;
  leftRearPID.ITerm = 0;

  rightRearPID.TargetTicksPerFrame = 0.0;
  rightRearPID.Encoder = readEncoder(RIGHT_REAR);
  rightRearPID.PrevEnc = rightRearPID.Encoder;
  rightRearPID.output = 0;
  rightRearPID.PrevInput = 0;
  rightRearPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo *p)
{
  long Perror;
  long output;
  int input;

  // Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  /*
   * Avoid derivative kick and allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  //  p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    /*
     * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
     */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID()
{
  /* Read the encoders */
  leftFrontPID.Encoder = readEncoder(LEFT_FRONT);
  rightFrontPID.Encoder = readEncoder(RIGHT_FRONT);
  leftRearPID.Encoder = readEncoder(LEFT_REAR);
  rightRearPID.Encoder = readEncoder(RIGHT_REAR);

  /* If we're not moving there is nothing more to do */
  if (!moving)
  {
    /*
     * Reset PIDs once, to prevent startup spikes,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
     * PrevInput is considered a good proxy to detect
     * whether reset has already happened
     */
    if (leftFrontPID.PrevInput != 0 || rightFrontPID.PrevInput != 0 || leftRearPID.PrevInput != 0 || rightRearPID.PrevInput != 0)
      resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightFrontPID);
  doPID(&leftFrontPID);
  doPID(&rightRearPID);
  doPID(&leftRearPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftFrontPID.output, rightFrontPID.output, leftRearPID.output, rightRearPID.output);
}
