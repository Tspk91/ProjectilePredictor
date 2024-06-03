//All in world space! Gets point you have to aim to
//NOTE: this will break with infinite speed projectiles!
//https://gamedev.stackexchange.com/questions/149327/projectile-aim-prediction-with-acceleration
public static Vector3 GetTargetLeadingPositionQuadratic(Vector3 launchPosition, Vector3 launcherVelocity, Vector3 projectileAcceleration, Vector3 targetPosition, Vector3 targetVelocity, Vector3 targetAcceleration, float distance, float projectileSpeed, int iterations)
{
    Vector3 pT = targetPosition - launchPosition;
    Vector3 vT = targetVelocity - launcherVelocity;
    Vector3 aT = targetAcceleration;
    float s = projectileSpeed;
    Vector3 aP = projectileAcceleration;

    Vector3 accel = aT - aP;

    //time to target guess
    float guess = distance / s;

    if (iterations > 0)
    {
        //quartic coefficients
        float a = Vector3.Dot(accel, accel) * 0.25f;
        float b = Vector3.Dot(accel, vT);
        float c = Vector3.Dot(accel, pT) + Vector3.Dot(vT, vT) - s * s;
        float d = 2f * Vector3.Dot(vT, pT);
        float e = Vector3.Dot(pT, pT);

        //solve with newton
        float finalGuess = SolveQuarticNewton(guess, iterations, a, b, c, d, e);

        //use first guess if negative or zero
        if (finalGuess > 0f)
        {
            guess = finalGuess;
        }
    }

    Vector3 travel = pT + vT * guess + 0.5f * aT * guess * guess;

    return launchPosition + travel;
}

//All in world space! Gets launch velocity you have to aim to
//NOTE: this will break with infinite speed projectiles!
//https://gamedev.stackexchange.com/questions/149327/projectile-aim-prediction-with-acceleration
public static Vector3 GetTargetLeadingVelocityQuadratic(Vector3 launchPosition, Vector3 launcherVelocity, Vector3 projectileAcceleration, Vector3 targetPosition, Vector3 targetVelocity, Vector3 targetAcceleration, float distance, float projectileSpeed, int iterations)
{
    Vector3 pT = targetPosition - launchPosition;
    Vector3 vT = targetVelocity - launcherVelocity;
    Vector3 aT = targetAcceleration;
    float s = projectileSpeed;
    Vector3 aP = projectileAcceleration;

    Vector3 accel = aT - aP;

    //time to target guess
    float guess = distance / s;

    if (iterations > 0)
    {
        //quartic coefficients
        float a = Vector3.Dot(accel, accel) * 0.25f;
        float b = Vector3.Dot(accel, vT);
        float c = Vector3.Dot(accel, pT) + Vector3.Dot(vT, vT) - s * s;
        float d = 2f * Vector3.Dot(vT, pT);
        float e = Vector3.Dot(pT, pT);

        //solve with newton
        float finalGuess = SolveQuarticNewton(guess, iterations, a, b, c, d, e);

        //use first guess if negative or zero
        if (finalGuess > 0f)
        {
            guess = finalGuess;
        }
    }

    Vector3 travel = pT + vT * guess + 0.5f * aT * guess * guess;

    Vector3 launchVelocity = travel / guess - 0.5f * aP * guess;

    return launchVelocity;
}

static float SolveQuarticNewton(float guess, int iterations, float a, float b, float c, float d, float e)
{
    for (int i = 0; i < iterations; i++)
    {
        guess = guess - EvalQuartic(guess, a, b, c, d, e) / EvalQuarticDerivative(guess, a, b, c, d);
    }
    return guess;
}

static float EvalQuartic(float t, float a, float b, float c, float d, float e)
{
    return a * t * t * t * t + b * t * t * t + c * t * t + d * t + e;
}

static float EvalQuarticDerivative(float t, float a, float b, float c, float d)
{
    return 4f * a * t * t * t + 3f * b * t * t + 2f * c * t + d;
}