package us.ihmc.euclid.tuple4D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class QuaternionTest extends QuaternionBasicsTest<Quaternion>
{
   public static final double EPS = 1e-14;

   @Test
   public void testQuaternion()
   {
      Random random = new Random(613615L);
      Quaternion quaternion = new Quaternion();
      Quaternion quaternionCopy;
      Quaternion expected = new Quaternion();

      { // Test Quaternion()
         expected.setToZero();
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(QuaternionBasics other)
         quaternion = quaternionCopy = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion quaternion2 = new Quaternion(quaternion);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, quaternion2, EPS);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(double x, double y, double z, double s)
         expected = EuclidCoreRandomTools.nextQuaternion(random);
         expected.normalizeAndLimitToPi();
         quaternion = new Quaternion(expected.getX(), expected.getY(), expected.getZ(), expected.getS());

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(double[] quaternionArray)
         expected = EuclidCoreRandomTools.nextQuaternion(random);

         double[] quaternionArray;
         quaternionArray = new double[] {expected.getX(), expected.getY(), expected.getZ(), expected.getS()};

         quaternion = new Quaternion(quaternionArray);

         EuclidCoreTestTools.assertQuaternionEquals(expected, quaternion, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(RotationMatrix rotationMatrix)
         RotationMatrix rotationMatrix, rotationMatrixCopy;
         rotationMatrix = rotationMatrixCopy = EuclidCoreRandomTools.nextRotationMatrix(random);

         quaternion = new Quaternion(rotationMatrix);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, expected);

         EuclidCoreTestTools.assertQuaternionEquals(expected, quaternion, EPS);
         EuclidCoreTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(VectorBasics rotationVector)
         Vector3D rotationVector, rotationVectorCopy;
         rotationVector = rotationVectorCopy = EuclidCoreRandomTools.nextRotationVector(random);

         quaternion = new Quaternion(rotationVector);
         QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, expected);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(rotationVector, rotationVectorCopy, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Quaternion(double yaw, double pitch, double roll)
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         quaternion = new Quaternion(yaw, pitch, roll);
         QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, expected);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);

      int newHashCode, previousHashCode;
      newHashCode = q.hashCode();
      assertEquals(newHashCode, q.hashCode());

      previousHashCode = q.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         double qx = q.getX();
         double qy = q.getY();
         double qz = q.getZ();
         double qs = q.getS();
         switch (random.nextInt(4))
         {
            case 0:
               qx = random.nextDouble();
               break;
            case 1:
               qy = random.nextDouble();
               break;
            case 2:
               qz = random.nextDouble();
               break;
            case 3:
               qs = random.nextDouble();
               break;
         }
         q.setUnsafe(qx, qy, qz, qs);
         newHashCode = q.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testInterpolateApprox()
   {
      Random random = new Random(453);

      Quaternion q0 = EuclidCoreRandomTools.nextQuaternion(random);
      Quaternion qf = EuclidCoreRandomTools.nextQuaternion(random);
      qf.appendPitchRotation(0.3);
      Quaternion expected = new Quaternion();
      Quaternion actual = new Quaternion();
      Vector4D difference = new Vector4D();

      System.out.println(q0.distance(qf));
      System.out.println(q0.dot(qf));

      for (double t = 0.0; t <= 1.0; t += 0.1)
      {
         interpolateApprox(q0, qf, t, actual);
         expected.interpolate(q0, qf, t);

         difference.sub(expected, actual);
         difference.absolute();

         System.out.println("alpha = " + t + ", error angle: " + expected.distance(actual) + ", diff: " + difference + ", err: " + difference.norm());
      }
   }

   @Test
   public void testBenchmarkInterpolateApprox()
   {
      Random random = new Random(453);

      Quaternion q0 = EuclidCoreRandomTools.nextQuaternion(random);
      Quaternion qf = EuclidCoreRandomTools.nextQuaternion(random);
      Quaternion actual = new Quaternion();
      Quaternion expected = new Quaternion();

      int iterations = 10000000;

      for (int i = 0; i < iterations; i++)
      {
         double t = i / (iterations - 1.0);
         interpolateApprox(q0, qf, t, actual);
         expected.interpolate(q0, qf, t);
      }

      long start, end;

      start = System.nanoTime();
      for (int i = 0; i < iterations; i++)
      {
         double t = i / (iterations - 1.0);
         interpolateApprox(q0, qf, t, actual);
      }
      end = System.nanoTime();

      System.out.println("Approx: " + ((end - start) * 1.0e6) + "millis for " + iterations + " iterations");

      start = System.nanoTime();
      for (int i = 0; i < iterations; i++)
      {
         double t = i / (iterations - 1.0);
         expected.interpolate(q0, qf, t);
      }
      end = System.nanoTime();
      System.out.println("Exact: " + ((end - start) * 1.0e6) + "millis for " + iterations + " iterations");

   }

   private static final double mu = 1.15479905903339; //1.85298109240830;
   private static final double u0 = 1.0 / (1.0 * 3.0);
   private static final double u1 = 1.0 / (2.0 * 5.0);
   private static final double u2 = 1.0 / (3.0 * 7.0);
   private static final double u3 = 1.0 / (4.0 * 9.0);
   private static final double u4 = 1.0 / (5.0 * 11.0);
   private static final double u5 = 1.0 / (6.0 * 13.0);
   private static final double u6 = 1.0 / (7.0 * 15.0);
   private static final double u7 = mu / (8.0 * 17.0);
   private static final double v0 = 1.0 / 3.0;
   private static final double v1 = 2.0 / 5.0;
   private static final double v2 = 3.0 / 7.0;
   private static final double v3 = 4.0 / 9.0;
   private static final double v4 = 5.0 / 11.0;
   private static final double v5 = 6.0 / 13.0;
   private static final double v6 = 7.0 / 15.0;
   private static final double v7 = mu * 8.0 / 17.0;

   private static void interpolateApprox(QuaternionReadOnly q0, QuaternionReadOnly qf, double alpha, QuaternionBasics out)
   {
      if (alpha == 0.0)
      {
         out.set(q0);
         return;
      }

      if (alpha == 1.0)
      {
         out.set(qf);
         return;
      }

      double q0x = q0.getX();
      double q0y = q0.getY();
      double q0z = q0.getZ();
      double q0s = q0.getS();

      double qfx = qf.getX();
      double qfy = qf.getY();
      double qfz = qf.getZ();
      double qfs = qf.getS();

      if (q0.dot(qf) < 0.0)
      {
         qfx = -qfx;
         qfy = -qfy;
         qfz = -qfz;
         qfs = -qfs;
      }

      double qmx = q0x + qfx;
      double qmy = q0y + qfy;
      double qmz = q0z + qfz;
      double qms = q0s + qfs;
      double normInv = 1.0 / EuclidCoreTools.norm(qmx, qmy, qmz, qms);
      qmx *= normInv;
      qmy *= normInv;
      qmz *= normInv;
      qms *= normInv;

      if (alpha == 0.5)
      {
         out.setUnsafe(qmx, qmy, qmz, qms);
      }
      else if (alpha < 0.5)
      {
         interpolateApprox2(q0x, q0y, q0z, q0s, qmx, qmy, qmz, qms, 2.0 * alpha, out);
      }
      else
      {
         interpolateApprox2(qmx, qmy, qmz, qms, qfx, qfy, qfz, qfs, 2.0 * alpha - 1.0, out);
      }
   }

   private static void interpolateApprox2(double q0x,
                                          double q0y,
                                          double q0z,
                                          double q0s,
                                          double qfx,
                                          double qfy,
                                          double qfz,
                                          double qfs,
                                          double alpha,
                                          QuaternionBasics out)
   {
      double dot = q0x * qfx + q0y * qfy + q0z * qfz + q0s * qfs;

      double dotNeg = dot - 1.0;
      double beta = 1.0 - alpha;
      double alphaSquare = alpha * alpha;
      double betaSquare = beta * beta;

      double a0 = (u0 * alphaSquare - v0) * dotNeg;
      double a1 = (u1 * alphaSquare - v1) * dotNeg;
      double a2 = (u2 * alphaSquare - v2) * dotNeg;
      double a3 = (u3 * alphaSquare - v3) * dotNeg;
      double a4 = (u4 * alphaSquare - v4) * dotNeg;
      double a5 = (u5 * alphaSquare - v5) * dotNeg;
      double a6 = (u6 * alphaSquare - v6) * dotNeg;
      double a7 = (u7 * alphaSquare - v7) * dotNeg;

      double b0 = (u0 * betaSquare - v0) * dotNeg;
      double b1 = (u1 * betaSquare - v1) * dotNeg;
      double b2 = (u2 * betaSquare - v2) * dotNeg;
      double b3 = (u3 * betaSquare - v3) * dotNeg;
      double b4 = (u4 * betaSquare - v4) * dotNeg;
      double b5 = (u5 * betaSquare - v5) * dotNeg;
      double b6 = (u6 * betaSquare - v6) * dotNeg;
      double b7 = (u7 * betaSquare - v7) * dotNeg;

      double alpha0 = beta * (1 + b0 * (1 + b1 * (1 + b2 * (1 + b3 * (1 + b4 * (1 + b5 * (1 + b6 * (1 + b7))))))));
      double alphaf = alpha * (1 + a0 * (1 + a1 * (1 + a2 * (1 + a3 * (1 + a4 * (1 + a5 * (1 + a6 * (1 + a7))))))));

      double x = alpha0 * q0x + alphaf * qfx;
      double y = alpha0 * q0y + alphaf * qfy;
      double z = alpha0 * q0z + alphaf * qfz;
      double s = alpha0 * q0s + alphaf * qfs;
      out.setUnsafe(x, y, z, s);
   }

   @Override
   public Quaternion createEmptyTuple()
   {
      return new Quaternion();
   }

   @Override
   public Quaternion createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.nextQuaternion(random);
   }

   @Override
   public Quaternion createTuple(double x, double y, double z, double s)
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setUnsafe(x, y, z, s);
      return quaternion;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-14;
   }
}
