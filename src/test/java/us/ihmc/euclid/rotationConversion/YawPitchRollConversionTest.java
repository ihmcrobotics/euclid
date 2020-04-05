package us.ihmc.euclid.rotationConversion;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public class YawPitchRollConversionTest
{
   private static final double EPSILON = 1.0e-12;
   private static final double MAX_PITCH_ANGLE = YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE - EPSILON;
   private static final double MIN_PITCH_ANGLE = YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE + EPSILON;

   @Test
   public void testMatrixToYawPitchRoll() throws Exception
   {
      RotationMatrix matrix = new RotationMatrix();
      YawPitchRoll actualYawPitchRoll = new YawPitchRoll();
      Vector3D actualEulerAngles = new Vector3D();

      double deltaAngle = 0.05 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = -Math.PI / 2.0; pitch <= Math.PI / 2.0; pitch += deltaAngle)
            {
               RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitch, roll, matrix);

               double actualYaw = YawPitchRollConversion.computeYawImpl(matrix.getM00(), matrix.getM10());
               assertEquals(yaw, actualYaw, EPSILON);

               double actualPitch = YawPitchRollConversion.computePitchImpl(matrix.getM20());
               assertEquals(pitch, actualPitch, EPSILON);

               double actualRoll = YawPitchRollConversion.computeRollImpl(matrix.getM21(), matrix.getM22());
               assertEquals(roll, actualRoll, EPSILON);

               actualYaw = YawPitchRollConversion.computeYaw(matrix);
               assertEquals(yaw, actualYaw, EPSILON);

               actualPitch = YawPitchRollConversion.computePitch(matrix);
               assertEquals(pitch, actualPitch, EPSILON);

               actualRoll = YawPitchRollConversion.computeRoll(matrix);
               assertEquals(roll, actualRoll, EPSILON);

               YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualYawPitchRoll);
               assertEquals(yaw, actualYawPitchRoll.getYaw(), EPSILON);
               assertEquals(pitch, actualYawPitchRoll.getPitch(), EPSILON);
               assertEquals(roll, actualYawPitchRoll.getRoll(), EPSILON);

               YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualEulerAngles);
               assertEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               assertEquals(roll, actualEulerAngles.getX(), EPSILON);
            }
         }
      }

      double actualYaw = YawPitchRollConversion.computeYawImpl(1.0, 0.0);
      assertTrue(actualYaw == 0.0);
      actualYaw = YawPitchRollConversion.computeYawImpl(Double.NaN, 0.0);
      assertTrue(Double.isNaN(actualYaw));
      actualYaw = YawPitchRollConversion.computeYawImpl(0.0, Double.NaN);
      assertTrue(Double.isNaN(actualYaw));

      double actualPitch = YawPitchRollConversion.computePitchImpl(0.0);
      assertTrue(actualPitch == 0.0);
      actualPitch = YawPitchRollConversion.computePitchImpl(Double.NaN);
      assertTrue(Double.isNaN(actualPitch));

      double actualRoll = YawPitchRollConversion.computeRollImpl(0.0, 1.0);
      assertTrue(actualRoll == 0.0);
      actualRoll = YawPitchRollConversion.computeRollImpl(Double.NaN, 0.0);
      assertTrue(Double.isNaN(actualRoll));
      actualRoll = YawPitchRollConversion.computeRollImpl(0.0, Double.NaN);
      assertTrue(Double.isNaN(actualRoll));
   }

   @Test
   public void testRotationScaleMatrixToYawPitchRoll() throws Exception
   {
      Random random = new Random(23423L);
      RotationScaleMatrix matrix = new RotationScaleMatrix();
      YawPitchRoll actualYawPitchRoll = new YawPitchRoll();
      Vector3D actualEulerAngles = new Vector3D();

      double deltaAngle = 0.05 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = -Math.PI / 2.0; pitch <= Math.PI / 2.0; pitch += deltaAngle)
            {
               matrix.setScale(EuclidCoreRandomTools.nextPoint3D(random, 0.0, 10.0));
               matrix.setYawPitchRoll(yaw, pitch, roll);

               double actualYaw = YawPitchRollConversion.computeYaw(matrix);
               assertEquals(yaw, actualYaw, EPSILON);

               double actualPitch = YawPitchRollConversion.computePitch(matrix);
               assertEquals(pitch, actualPitch, EPSILON);

               double actualRoll = YawPitchRollConversion.computeRoll(matrix);
               assertEquals(roll, actualRoll, EPSILON);

               YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualYawPitchRoll);
               assertEquals(yaw, actualYawPitchRoll.getYaw(), EPSILON);
               assertEquals(pitch, actualYawPitchRoll.getPitch(), EPSILON);
               assertEquals(roll, actualYawPitchRoll.getRoll(), EPSILON);

               YawPitchRollConversion.convertMatrixToYawPitchRoll(matrix, actualEulerAngles);
               assertEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               assertEquals(roll, actualEulerAngles.getX(), EPSILON);
            }
         }
      }

      double actualYaw = YawPitchRollConversion.computeYawImpl(1.0, 0.0);
      assertTrue(actualYaw == 0.0);
      actualYaw = YawPitchRollConversion.computeYawImpl(Double.NaN, 0.0);
      assertTrue(Double.isNaN(actualYaw));
      actualYaw = YawPitchRollConversion.computeYawImpl(0.0, Double.NaN);
      assertTrue(Double.isNaN(actualYaw));

      double actualPitch = YawPitchRollConversion.computePitchImpl(0.0);
      assertTrue(actualPitch == 0.0);
      actualPitch = YawPitchRollConversion.computePitchImpl(Double.NaN);
      assertTrue(Double.isNaN(actualPitch));

      double actualRoll = YawPitchRollConversion.computeRollImpl(0.0, 1.0);
      assertTrue(actualRoll == 0.0);
      actualRoll = YawPitchRollConversion.computeRollImpl(Double.NaN, 0.0);
      assertTrue(Double.isNaN(actualRoll));
      actualRoll = YawPitchRollConversion.computeRollImpl(0.0, Double.NaN);
      assertTrue(Double.isNaN(actualRoll));
   }

   @Test
   public void testQuaternionToYawPitchRoll() throws Exception
   {
      Quaternion quaternion = new Quaternion();
      YawPitchRoll actualYawPitchRoll = new YawPitchRoll();
      Vector3D actualEulerAngles = new Vector3D();

      double deltaAngle = 0.05 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
            {
               double pitch = EuclidCoreTools.interpolate(MIN_PITCH_ANGLE, MAX_PITCH_ANGLE, alpha);
               assertQuaternionToYawPitchRoll(yaw, pitch, roll, EPSILON);
            }

            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
            {
               double epsilon = EuclidCoreTools.interpolate(EPSILON, 1.0e-3, alpha);
               double pitch = EuclidCoreTools.interpolate(MAX_PITCH_ANGLE, Math.PI / 2.0 - EPSILON, alpha);
               assertQuaternionToYawPitchRoll(yaw, pitch, roll, epsilon);
            }

            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
            {
               double epsilon = EuclidCoreTools.interpolate(EPSILON, 1.0e-3, alpha);
               double pitch = EuclidCoreTools.interpolate(MIN_PITCH_ANGLE, -Math.PI / 2.0 + EPSILON, alpha);
               assertQuaternionToYawPitchRoll(yaw, pitch, roll, epsilon);
            }
         }
      }

      Random random = new Random(239478L);

      // Test with a non-unitary quaternion
      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = MIN_PITCH_ANGLE; pitch <= MAX_PITCH_ANGLE; pitch += deltaAngle)
            {
               QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, quaternion);
               double randomScale = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
               quaternion.setUnsafe(quaternion.getX() * randomScale,
                                    quaternion.getY() * randomScale,
                                    quaternion.getZ() * randomScale,
                                    quaternion.getS() * randomScale);

               double actualYaw = YawPitchRollConversion.computeYaw(quaternion);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

               double actualPitch = YawPitchRollConversion.computePitch(quaternion);
               assertEquals(pitch, actualPitch, EPSILON);

               double actualRoll = YawPitchRollConversion.computeRoll(quaternion);
               EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

               YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll.getYaw(), EPSILON);
               assertEquals(pitch, actualYawPitchRoll.getPitch(), EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll.getRoll(), EPSILON);

               YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), EPSILON);
            }
         }
      }

      // Test the checks on the norm
      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);

      double actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(actualYaw == 0.0);

      double actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(actualPitch == 0.0);

      double actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(actualRoll == 0.0);

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(actualYawPitchRoll.getYaw() == 0.0);
      assertTrue(actualYawPitchRoll.getPitch() == 0.0);
      assertTrue(actualYawPitchRoll.getRoll() == 0.0);

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(actualEulerAngles);

      // Test the checks on NaNs
      quaternion.setUnsafe(Double.NaN, 0.0, 0.0, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      quaternion.setUnsafe(0.0, Double.NaN, 0.0, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      quaternion.setUnsafe(0.0, 0.0, Double.NaN, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      quaternion.setUnsafe(0.0, 0.0, 0.0, Double.NaN);

      actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);
   }

   public void assertQuaternionToYawPitchRoll(double yaw, double pitch, double roll, double epsilon)
   {
      Quaternion quaternion = new Quaternion();
      YawPitchRoll actualYawPitchRoll = new YawPitchRoll();
      Vector3D actualEulerAngles = new Vector3D();

      QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, quaternion);

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double actualYaw = YawPitchRollConversion.computeYawFromQuaternionImpl(qx, qy, qz, qs);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, epsilon);

      double actualPitch = YawPitchRollConversion.computePitchFromQuaternionImpl(qx, qy, qz, qs);
      assertEquals(pitch, actualPitch, epsilon);

      double actualRoll = YawPitchRollConversion.computeRollFromQuaternionImpl(qx, qy, qz, qs);
      EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, epsilon);

      actualYaw = YawPitchRollConversion.computeYaw(quaternion);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, epsilon);

      actualPitch = YawPitchRollConversion.computePitch(quaternion);
      assertEquals(pitch, actualPitch, epsilon);

      actualRoll = YawPitchRollConversion.computeRoll(quaternion);
      EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, epsilon);

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualYawPitchRoll);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll.getYaw(), epsilon);
      assertEquals(pitch, actualYawPitchRoll.getPitch(), epsilon);
      EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll.getRoll(), epsilon);

      YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, actualEulerAngles);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), epsilon);
      assertEquals(pitch, actualEulerAngles.getY(), epsilon);
      EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), epsilon);
   }

   @Test
   public void testAxisAngleToYawPitchRoll() throws Exception
   {
      AxisAngle axisAngle = new AxisAngle();
      YawPitchRoll actualYawPitchRoll = new YawPitchRoll();
      Vector3D actualEulerAngles = new Vector3D();

      double deltaAngle = 0.05 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
            {
               double pitch = EuclidCoreTools.interpolate(MIN_PITCH_ANGLE, MAX_PITCH_ANGLE, alpha);
               assertAxisAngleToYawPitchRoll(yaw, pitch, roll, EPSILON);
            }

            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
            {
               double epsilon = EuclidCoreTools.interpolate(EPSILON, 1.0e-3, alpha);
               double pitch = EuclidCoreTools.interpolate(MAX_PITCH_ANGLE, Math.PI / 2.0 - EPSILON, alpha);
               assertAxisAngleToYawPitchRoll(yaw, pitch, roll, epsilon);
            }

            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
            {
               double epsilon = EuclidCoreTools.interpolate(EPSILON, 1.0e-3, alpha);
               double pitch = EuclidCoreTools.interpolate(MIN_PITCH_ANGLE, -Math.PI / 2.0 + EPSILON, alpha);
               assertAxisAngleToYawPitchRoll(yaw, pitch, roll, epsilon);
            }
         }
      }

      Random random = new Random(239478L);

      // Test with a non-unitary axes
      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = MIN_PITCH_ANGLE; pitch <= MAX_PITCH_ANGLE; pitch += deltaAngle)
            {
               AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, axisAngle);
               double randomScale = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
               axisAngle.setX(axisAngle.getX() * randomScale);
               axisAngle.setY(axisAngle.getY() * randomScale);
               axisAngle.setZ(axisAngle.getZ() * randomScale);

               double actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, EPSILON);

               double actualPitch = YawPitchRollConversion.computePitch(axisAngle);
               assertEquals(pitch, actualPitch, EPSILON);

               double actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
               EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, EPSILON);

               YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll.getYaw(), EPSILON);
               assertEquals(pitch, actualYawPitchRoll.getPitch(), EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll.getRoll(), EPSILON);

               YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
               EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), EPSILON);
               assertEquals(pitch, actualEulerAngles.getY(), EPSILON);
               EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), EPSILON);
            }
         }
      }

      // Test the checks on NaNs
      axisAngle.set(Double.NaN, 0.0, 0.0, 1.0);

      double actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      assertTrue(Double.isNaN(actualYaw));

      double actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertTrue(Double.isNaN(actualPitch));

      double actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      axisAngle.set(0.0, Double.NaN, 0.0, 1.0);

      actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      axisAngle.set(0.0, 0.0, Double.NaN, 1.0);

      actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      axisAngle.set(0.0, 0.0, 0.0, Double.NaN);

      actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);
   }

   public void assertAxisAngleToYawPitchRoll(double yaw, double pitch, double roll, double epsilon)
   {
      AxisAngle axisAngle = new AxisAngle();
      YawPitchRoll actualYawPitchRoll = new YawPitchRoll();
      Vector3D actualEulerAngles = new Vector3D();

      AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, axisAngle);

      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      double actualYaw = YawPitchRollConversion.computeYawFromAxisAngleImpl(ux, uy, uz, angle);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, epsilon);

      double actualPitch = YawPitchRollConversion.computePitchFromAxisAngleImpl(ux, uy, uz, angle);
      assertEquals(pitch, actualPitch, epsilon);

      double actualRoll = YawPitchRollConversion.computeRollFromAxisAngleImpl(ux, uy, uz, angle);
      EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, epsilon);

      actualYaw = YawPitchRollConversion.computeYaw(axisAngle);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, epsilon);

      actualPitch = YawPitchRollConversion.computePitch(axisAngle);
      assertEquals(pitch, actualPitch, epsilon);

      actualRoll = YawPitchRollConversion.computeRoll(axisAngle);
      EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, epsilon);

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualYawPitchRoll);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll.getYaw(), epsilon);
      assertEquals(pitch, actualYawPitchRoll.getPitch(), epsilon);
      EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll.getRoll(), epsilon);

      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, actualEulerAngles);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), epsilon);
      assertEquals(pitch, actualEulerAngles.getY(), epsilon);
      EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), epsilon);
   }

   @Test
   public void testRotationVectorToYawPitchRoll() throws Exception
   {
      Vector3D rotationVector = new Vector3D();
      YawPitchRoll actualYawPitchRoll = new YawPitchRoll();
      Vector3D actualEulerAngles = new Vector3D();

      double deltaAngle = 0.05 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
            {
               double pitch = EuclidCoreTools.interpolate(MIN_PITCH_ANGLE, MAX_PITCH_ANGLE, alpha);
               assertRotationVectorToYawPitchRoll(yaw, pitch, roll, EPSILON);
            }

            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
            {
               double epsilon = EuclidCoreTools.interpolate(EPSILON, 1.0e-3, alpha);
               double pitch = EuclidCoreTools.interpolate(MAX_PITCH_ANGLE, Math.PI / 2.0 - EPSILON, alpha);
               assertRotationVectorToYawPitchRoll(yaw, pitch, roll, epsilon);
            }

            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
            {
               double epsilon = EuclidCoreTools.interpolate(EPSILON, 1.0e-3, alpha);
               double pitch = EuclidCoreTools.interpolate(MIN_PITCH_ANGLE, -Math.PI / 2.0 + EPSILON, alpha);
               assertRotationVectorToYawPitchRoll(yaw, pitch, roll, epsilon);
            }
         }
      }

      // Test the checks on the norm
      rotationVector.setX(0.0);
      rotationVector.setY(0.0);
      rotationVector.setZ(0.0);

      double actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
      assertTrue(actualYaw == 0.0);

      double actualPitch = YawPitchRollConversion.computePitch(rotationVector);
      assertTrue(actualPitch == 0.0);

      double actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
      assertTrue(actualRoll == 0.0);

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
      assertTrue(actualYawPitchRoll.getYaw() == 0.0);
      assertTrue(actualYawPitchRoll.getPitch() == 0.0);
      assertTrue(actualYawPitchRoll.getRoll() == 0.0);

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(actualEulerAngles);

      // Test the checks on NaNs
      rotationVector.set(Double.NaN, 0.0, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(rotationVector);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      rotationVector.set(0.0, Double.NaN, 0.0);

      actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(rotationVector);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);

      rotationVector.set(0.0, 0.0, Double.NaN);

      actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
      assertTrue(Double.isNaN(actualYaw));

      actualPitch = YawPitchRollConversion.computePitch(rotationVector);
      assertTrue(Double.isNaN(actualPitch));

      actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
      assertTrue(Double.isNaN(actualRoll));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
      assertTrue(Double.isNaN(actualYawPitchRoll.getYaw()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getPitch()));
      assertTrue(Double.isNaN(actualYawPitchRoll.getRoll()));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualEulerAngles);
   }

   public void assertRotationVectorToYawPitchRoll(double yaw, double pitch, double roll, double epsilon)
   {
      Vector3D rotationVector = new Vector3D();
      Vector3D rotationVectorCopy = new Vector3D();
      YawPitchRoll actualYawPitchRoll = new YawPitchRoll();
      Vector3D actualEulerAngles = new Vector3D();

      RotationVectorConversion.convertYawPitchRollToRotationVector(yaw, pitch, roll, rotationVector);
      rotationVectorCopy.set(rotationVector);

      double actualYaw = YawPitchRollConversion.computeYaw(rotationVector);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualYaw, epsilon);
      assertTrue(rotationVector.equals(rotationVectorCopy));

      double actualPitch = YawPitchRollConversion.computePitch(rotationVector);
      assertEquals(pitch, actualPitch, epsilon);
      assertTrue(rotationVector.equals(rotationVectorCopy));

      double actualRoll = YawPitchRollConversion.computeRoll(rotationVector);
      EuclidCoreTestTools.assertAngleEquals(roll, actualRoll, epsilon);
      assertTrue(rotationVector.equals(rotationVectorCopy));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualYawPitchRoll);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualYawPitchRoll.getYaw(), epsilon);
      assertEquals(pitch, actualYawPitchRoll.getPitch(), epsilon);
      EuclidCoreTestTools.assertAngleEquals(roll, actualYawPitchRoll.getRoll(), epsilon);
      assertTrue(rotationVector.equals(rotationVectorCopy));

      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(rotationVector, actualEulerAngles);
      EuclidCoreTestTools.assertAngleEquals(yaw, actualEulerAngles.getZ(), epsilon);
      assertEquals(pitch, actualEulerAngles.getY(), epsilon);
      EuclidCoreTestTools.assertAngleEquals(roll, actualEulerAngles.getX(), epsilon);
      assertTrue(rotationVector.equals(rotationVectorCopy));
   }
}
