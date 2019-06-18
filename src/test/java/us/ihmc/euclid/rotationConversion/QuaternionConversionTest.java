package us.ihmc.euclid.rotationConversion;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class QuaternionConversionTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testAxisAngleToQuaternion() throws Exception
   {
      Random random = new Random(484514L);
      double minMaxAngleRange = 2.0 * Math.PI;
      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();

      for (int i = 0; i < ITERATIONS; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
         double angle = axisAngle.getAngle();
         double ux = axisAngle.getX();
         double uy = axisAngle.getY();
         double uz = axisAngle.getZ();
         // As the axis-angle is sane, there is no edge case making the conversion straightforward.
         double qs = Math.cos(angle / 2.0);
         double qx = ux * Math.sin(angle / 2.0);
         double qy = uy * Math.sin(angle / 2.0);
         double qz = uz * Math.sin(angle / 2.0);
         expectedQuaternion.setUnsafe(qx, qy, qz, qs);

         QuaternionConversion.convertAxisAngleToQuaternion(ux, uy, uz, angle, actualQuaternion);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
      }

      // Test with an axis-angle that has a non unnitary axis.
      double scale = random.nextDouble();
      AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
      double angle = axisAngle.getAngle();
      double ux = axisAngle.getX();
      double uy = axisAngle.getY();
      double uz = axisAngle.getZ();
      QuaternionConversion.convertAxisAngleToQuaternion(ux, uy, uz, angle, expectedQuaternion);
      QuaternionConversion.convertAxisAngleToQuaternion(scale * ux, scale * uy, scale * uz, angle, actualQuaternion);
      EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
      EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);

      QuaternionConversion.convertAxisAngleToQuaternion(0.0, 0.0, 0.0, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertQuaternionIsSetToZero(actualQuaternion);

      QuaternionConversion.convertAxisAngleToQuaternion(Double.NaN, 0.0, 0.0, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertAxisAngleToQuaternion(0.0, Double.NaN, 0.0, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertAxisAngleToQuaternion(0.0, 0.0, Double.NaN, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertAxisAngleToQuaternion(0.0, 0.0, 0.0, Double.NaN, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      // Test with an actual quaternion
      for (int i = 0; i < 100; i++)
      {
         axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
         AxisAngle axisAngleCopy = new AxisAngle(axisAngle);
         angle = axisAngle.getAngle();
         ux = axisAngle.getX();
         uy = axisAngle.getY();
         uz = axisAngle.getZ();
         QuaternionConversion.convertAxisAngleToQuaternion(ux, uy, uz, angle, expectedQuaternion);
         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, actualQuaternion);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
         // Assert that the parameter does not get modified
         assertTrue(axisAngle.equals(axisAngleCopy));
      }
   }

   @Test
   public void testMatrixToQuaternion() throws Exception
   {
      Random random = new Random(2135L);
      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();
      RotationMatrix rotationMatrix = new RotationMatrix();
      double minMaxAngleRange = Math.PI;
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;

      for (int i = 0; i < ITERATIONS; i++)
      {
         expectedQuaternion = EuclidCoreRandomTools.nextQuaternion(random, minMaxAngleRange);
         double qx = expectedQuaternion.getX();
         double qy = expectedQuaternion.getY();
         double qz = expectedQuaternion.getZ();
         double qs = expectedQuaternion.getS();

         // The quaternion is 'sane' and the conversion to a matrix is simple (no edge case).
         // See Wikipedia for the conversion: https://en.wikipedia.org/wiki/Rotation_matrix
         m00 = 1.0 - 2.0 * (qy * qy + qz * qz);
         m11 = 1.0 - 2.0 * (qx * qx + qz * qz);
         m22 = 1.0 - 2.0 * (qx * qx + qy * qy);

         m01 = 2.0 * (qx * qy - qz * qs);
         m10 = 2.0 * (qx * qy + qz * qs);

         m20 = 2.0 * (qx * qz - qy * qs);
         m02 = 2.0 * (qx * qz + qy * qs);

         m12 = 2.0 * (qy * qz - qx * qs);
         m21 = 2.0 * (qy * qz + qx * qs);

         rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);

         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D randomVector = EuclidCoreRandomTools.nextVector3D(random);
         randomVector.normalize();
         expectedQuaternion.setUnsafe(randomVector.getX(), randomVector.getY(), randomVector.getZ(), 0.0); // rotation angle of Pi
         double qx = expectedQuaternion.getX();
         double qy = expectedQuaternion.getY();
         double qz = expectedQuaternion.getZ();
         double qs = expectedQuaternion.getS();

         // The quaternion is 'sane' and the conversion to a matrix is simple (no edge case).
         // See Wikipedia for the conversion: https://en.wikipedia.org/wiki/Rotation_matrix
         m00 = 1.0 - 2.0 * (qy * qy + qz * qz);
         m11 = 1.0 - 2.0 * (qx * qx + qz * qz);
         m22 = 1.0 - 2.0 * (qx * qx + qy * qy);

         m01 = 2.0 * (qx * qy - qz * qs);
         m10 = 2.0 * (qx * qy + qz * qs);

         m20 = 2.0 * (qx * qz - qy * qs);
         m02 = 2.0 * (qx * qz + qy * qs);

         m12 = 2.0 * (qy * qz - qx * qs);
         m21 = 2.0 * (qy * qz + qx * qs);

         rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
      }

      double sqrt2Over2 = Math.sqrt(2.0) / 2.0;
      // Test edge cases
      // Zero rotation
      m00 = m11 = m22 = 1.0;
      m01 = m02 = m12 = 0.0;
      m10 = m20 = m21 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertQuaternionIsSetToZero(actualQuaternion);

      // Pi/2 around x
      m00 = 1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = 0.0;
      m12 = -1.0;
      m20 = 0.0;
      m21 = 1.0;
      m22 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      assertEquals(sqrt2Over2, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getS(), EPSILON);

      // Pi around x
      m00 = 1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = -1.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = -1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      assertEquals(1.0, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi/2 around y
      m00 = 0.0;
      m01 = 0.0;
      m02 = 1.0;
      m10 = 0.0;
      m11 = 1.0;
      m12 = 0.0;
      m20 = -1.0;
      m21 = 0.0;
      m22 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getS(), EPSILON);

      // Pi around z
      m00 = -1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = 1.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = -1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(1.0, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi/2 around z
      m00 = 0.0;
      m01 = -1.0;
      m02 = 0.0;
      m10 = 1.0;
      m11 = 0.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = 1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getZ(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getS(), EPSILON);

      // Pi around z
      m00 = -1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = -1.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = 1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(1.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi around xy (as axis-angle: (x = sqrt(2)/2, y = sqrt(2)/2, z = 0, angle = Pi)
      m00 = 0.0;
      m01 = 1.0;
      m02 = 0.0;
      m10 = 1.0;
      m11 = 0.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = -1.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      assertEquals(sqrt2Over2, actualQuaternion.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getY(), EPSILON);
      assertEquals(0.0, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi around xz (as axis-angle: (x = sqrt(2)/2, y = 0, z = sqrt(2)/2, angle = Pi)
      m00 = 0.0;
      m01 = 0.0;
      m02 = 1.0;
      m10 = 0.0;
      m11 = -1.0;
      m12 = 0.0;
      m20 = 1.0;
      m21 = 0.0;
      m22 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      assertEquals(sqrt2Over2, actualQuaternion.getX(), EPSILON);
      assertEquals(0.0, actualQuaternion.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      // Pi around yz (as axis-angle: (x = 0, y = sqrt(2)/2, z = sqrt(2)/2, angle = Pi)
      m00 = -1.0;
      m01 = 0.0;
      m02 = 0.0;
      m10 = 0.0;
      m11 = 0.0;
      m12 = 1.0;
      m20 = 0.0;
      m21 = 1.0;
      m22 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      assertEquals(0.0, actualQuaternion.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualQuaternion.getZ(), EPSILON);
      assertEquals(0.0, actualQuaternion.getS(), EPSILON);

      rotationMatrix.setUnsafe(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);
      rotationMatrix.setUnsafe(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);
      rotationMatrix.setUnsafe(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN);
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      // Test with a RotationScaleMatrix
      for (int i = 0; i < 1000; i++)
      {
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationScaleMatrix rotationScaleMatrixCopy = new RotationScaleMatrix(rotationScaleMatrix);
         QuaternionConversion.convertMatrixToQuaternion(rotationScaleMatrix, actualQuaternion);
         QuaternionConversion.convertMatrixToQuaternion(rotationScaleMatrix.getRotationMatrix(), expectedQuaternion);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
         // Assert the parameter does not get modified
         assertTrue(rotationScaleMatrix.equals(rotationScaleMatrixCopy));
      }
   }

   @Test
   public void testYawPitchRollToQuaternion() throws Exception
   {
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;
      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();
      RotationMatrix rotationMatrix = new RotationMatrix();

      double deltaAngle = 0.1 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double pitch = -Math.PI / 2.0; pitch <= Math.PI / 2.0; pitch += deltaAngle)
         {
            for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
            {
               double cYaw = Math.cos(yaw);
               double sYaw = Math.sin(yaw);
               double cPitch = Math.cos(pitch);
               double sPitch = Math.sin(pitch);
               double cRoll = Math.cos(roll);
               double sRoll = Math.sin(roll);

               m00 = cYaw * cPitch;
               m01 = cYaw * sPitch * sRoll - sYaw * cRoll;
               m02 = cYaw * sPitch * cRoll + sYaw * sRoll;
               m10 = sYaw * cPitch;
               m11 = sYaw * sPitch * sRoll + cYaw * cRoll;
               m12 = sYaw * sPitch * cRoll - cYaw * sRoll;
               m20 = -sPitch;
               m21 = cPitch * sRoll;
               m22 = cPitch * cRoll;

               rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
               QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, expectedQuaternion);
               QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, actualQuaternion);
               EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, EPSILON);
               EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
            }
         }
      }

      QuaternionConversion.convertYawPitchRollToQuaternion(0.0, 0.0, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertQuaternionIsSetToZero(actualQuaternion);

      QuaternionConversion.convertYawPitchRollToQuaternion(Double.NaN, 0.0, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertYawPitchRollToQuaternion(0.0, Double.NaN, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertYawPitchRollToQuaternion(0.0, 0.0, Double.NaN, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);
   }

   @Test
   public void testRotationVectorToQuaternion() throws Exception
   {
      Random random = new Random(32047230L);
      double minMaxAngleRange = 2.0 * Math.PI;
      Quaternion expectedQuaternion = new Quaternion();
      Quaternion actualQuaternion = new Quaternion();

      for (int i = 0; i < ITERATIONS; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
         double rx = axisAngle.getX() * axisAngle.getAngle();
         double ry = axisAngle.getY() * axisAngle.getAngle();
         double rz = axisAngle.getZ() * axisAngle.getAngle();
         // The axisangle->quaternion conversion is safe here as it is tested separately in testAxisAngleToQuaternion().
         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, expectedQuaternion);
         QuaternionConversion.convertRotationVectorToQuaternion(rx, ry, rz, actualQuaternion);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
      }

      QuaternionConversion.convertRotationVectorToQuaternion(0.0, 0.0, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertQuaternionIsSetToZero(actualQuaternion);

      QuaternionConversion.convertRotationVectorToQuaternion(Double.NaN, 0.0, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertRotationVectorToQuaternion(0.0, Double.NaN, 0.0, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      QuaternionConversion.convertRotationVectorToQuaternion(0.0, 0.0, Double.NaN, actualQuaternion);
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(actualQuaternion);

      // Test with an actual vector
      Vector3D rotationVector = new Vector3D();
      Vector3D rotationVectorCopy = new Vector3D();

      for (int i = 0; i < 1000; i++)
      {
         EuclidCoreRandomTools.randomizeTuple3D(random, new Point3D(minMaxAngleRange, minMaxAngleRange, minMaxAngleRange), rotationVector);
         rotationVectorCopy.set(rotationVector);

         double rx = rotationVector.getX();
         double ry = rotationVector.getY();
         double rz = rotationVector.getZ();
         QuaternionConversion.convertRotationVectorToQuaternion(rx, ry, rz, expectedQuaternion);
         QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, actualQuaternion);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
         // Assert that the parameter does not get modified
         assertTrue(rotationVector.equals(rotationVectorCopy));
      }

      // test some very small angles
      minMaxAngleRange = 1.0e-8;
      for (int i = 0; i < 10000; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
         double rx = axisAngle.getX() * axisAngle.getAngle();
         double ry = axisAngle.getY() * axisAngle.getAngle();
         double rz = axisAngle.getZ() * axisAngle.getAngle();
         // The axisangle->quaternion conversion is safe here as it is tested separately in testAxisAngleToQuaternion().
         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, expectedQuaternion);
         QuaternionConversion.convertRotationVectorToQuaternion(rx, ry, rz, actualQuaternion);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPSILON);
         EuclidCoreTestTools.assertQuaternionIsUnitary(actualQuaternion, EPSILON);
      }
   }
}
