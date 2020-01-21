package us.ihmc.euclid.rotationConversion;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;
import static us.ihmc.euclid.tools.EuclidCoreTools.cos;
import static us.ihmc.euclid.tools.EuclidCoreTools.sin;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class AxisAngleConversionTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testQuaternionToAxisAngle() throws Exception
   {
      Random random = new Random(51651L);
      AxisAngle axisAngle = new AxisAngle();
      Quaternion quaternion = new Quaternion();

      for (int i = 0; i < ITERATIONS; i++)
      {
         double ux = EuclidCoreRandomTools.nextDouble(random);
         double uy = EuclidCoreRandomTools.nextDouble(random);
         double uz = EuclidCoreRandomTools.nextDouble(random);

         double norm = EuclidCoreTools.norm(ux, uy, uz);
         ux /= norm;
         uy /= norm;
         uz /= norm;
         double angle = EuclidCoreRandomTools.nextDouble(random, 2.0 * Math.PI);

         double qs = EuclidCoreTools.cos(angle / 2.0);
         double qx = ux * EuclidCoreTools.sin(angle / 2.0);
         double qy = uy * EuclidCoreTools.sin(angle / 2.0);
         double qz = uz * EuclidCoreTools.sin(angle / 2.0);
         quaternion.setUnsafe(qx, qy, qz, qs);
         AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);

         if (axisAngle.getAngle() * angle < 0.0)
         {
            axisAngle.negate();
         }

         assertEquals(ux, axisAngle.getX(), EPSILON);
         assertEquals(uy, axisAngle.getY(), EPSILON);
         assertEquals(uz, axisAngle.getZ(), EPSILON);
         assertEquals(angle, axisAngle.getAngle(), EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(axisAngle, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AxisAngle originalAxisAngle = new AxisAngle();
         EuclidCoreRandomTools.randomizeAxisAngle(random, 2.0 * Math.PI, originalAxisAngle);

         double qs = EuclidCoreTools.cos(originalAxisAngle.getAngle() / 2.0);
         double qx = originalAxisAngle.getX() * EuclidCoreTools.sin(originalAxisAngle.getAngle() / 2.0);
         double qy = originalAxisAngle.getY() * EuclidCoreTools.sin(originalAxisAngle.getAngle() / 2.0);
         double qz = originalAxisAngle.getZ() * EuclidCoreTools.sin(originalAxisAngle.getAngle() / 2.0);
         quaternion.setUnsafe(qx, qy, qz, qs);
         AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);

         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(originalAxisAngle, axisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(axisAngle, EPSILON);
      }

      // Test with a quaternion that is not unitary
      double ux = EuclidCoreRandomTools.nextDouble(random);
      double uy = EuclidCoreRandomTools.nextDouble(random);
      double uz = EuclidCoreRandomTools.nextDouble(random);

      double norm = EuclidCoreTools.norm(ux, uy, uz);
      ux /= norm;
      uy /= norm;
      uz /= norm;
      double angle = EuclidCoreRandomTools.nextDouble(random, 2.0 * Math.PI);
      double scale = random.nextDouble();

      double qs = scale * EuclidCoreTools.cos(angle / 2.0);
      double qx = scale * ux * EuclidCoreTools.sin(angle / 2.0);
      double qy = scale * uy * EuclidCoreTools.sin(angle / 2.0);
      double qz = scale * uz * EuclidCoreTools.sin(angle / 2.0);
      quaternion.setUnsafe(qx, qy, qz, qs);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);

      if (axisAngle.getAngle() * angle < 0.0)
      {
         axisAngle.negate();
      }

      assertEquals(ux, axisAngle.getX(), EPSILON);
      assertEquals(uy, axisAngle.getY(), EPSILON);
      assertEquals(uz, axisAngle.getZ(), EPSILON);
      assertEquals(angle, axisAngle.getAngle(), EPSILON);
      EuclidCoreTestTools.assertAxisUnitary(axisAngle, EPSILON);

      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleIsSetToZero(axisAngle);

      quaternion.setUnsafe(0.0, 0.0, 0.0, Double.NaN);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      quaternion.setUnsafe(0.0, 0.0, Double.NaN, 0.0);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      quaternion.setUnsafe(0.0, Double.NaN, 0.0, 0.0);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      quaternion.setUnsafe(Double.NaN, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, axisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);
   }

   @Test
   public void testRotationVectorToAxisAngle() throws Exception
   {
      Random random = new Random(2135L);
      AxisAngle expectedAxisAngle = new AxisAngle();
      AxisAngle actualAxisAngle = new AxisAngle();
      double minMaxAngleRange = 2.0 * Math.PI;

      for (int i = 0; i < ITERATIONS; i++)
      {
         EuclidCoreRandomTools.randomizeAxisAngle(random, minMaxAngleRange, expectedAxisAngle);
         double rx = expectedAxisAngle.getX() * expectedAxisAngle.getAngle();
         double ry = expectedAxisAngle.getY() * expectedAxisAngle.getAngle();
         double rz = expectedAxisAngle.getZ() * expectedAxisAngle.getAngle();
         AxisAngleConversion.convertRotationVectorToAxisAngle(rx, ry, rz, actualAxisAngle);

         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
      }

      AxisAngleConversion.convertRotationVectorToAxisAngle(0.0, 0.0, 0.0, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleIsSetToZero(actualAxisAngle);

      AxisAngleConversion.convertRotationVectorToAxisAngle(Double.NaN, 0.0, 0.0, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      AxisAngleConversion.convertRotationVectorToAxisAngle(0.0, Double.NaN, 0.0, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      AxisAngleConversion.convertRotationVectorToAxisAngle(0.0, 0.0, Double.NaN, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      // Test with an actual vector
      for (int i = 0; i < 1000; i++)
      {
         Vector3D rotationVector = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D rotationVectorCopy = new Vector3D(rotationVector);
         AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ(), expectedAxisAngle);
         AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector, actualAxisAngle);

         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
         // Assert that the parameter does not get modified
         assertTrue(rotationVector.equals(rotationVectorCopy));
      }
   }

   @Test
   public void testMatrixToAxisAngle() throws Exception
   {
      Random random = new Random(2135L);
      AxisAngle expectedAxisAngle = new AxisAngle();
      AxisAngle actualAxisAngle = new AxisAngle();
      double minMaxAngleRange = Math.PI;
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;
      RotationMatrix rotationMatrix = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         EuclidCoreRandomTools.randomizeAxisAngle(random, minMaxAngleRange, expectedAxisAngle);
         double ux = expectedAxisAngle.getX();
         double uy = expectedAxisAngle.getY();
         double uz = expectedAxisAngle.getZ();
         double angle = expectedAxisAngle.getAngle();

         // The axis angle is 'sane' and the conversion to a matrix is simple (no edge case).
         // See Wikipedia for the conversion: https://en.wikipedia.org/wiki/Rotation_matrix
         m00 = cos(angle) + ux * ux * (1.0 - cos(angle));
         m11 = cos(angle) + uy * uy * (1.0 - cos(angle));
         m22 = cos(angle) + uz * uz * (1.0 - cos(angle));

         m01 = ux * uy * (1.0 - cos(angle)) - uz * sin(angle);
         m10 = ux * uy * (1.0 - cos(angle)) + uz * sin(angle);

         m20 = ux * uz * (1.0 - cos(angle)) - uy * sin(angle);
         m02 = ux * uz * (1.0 - cos(angle)) + uy * sin(angle);

         m12 = uy * uz * (1.0 - cos(angle)) - ux * sin(angle);
         m21 = uy * uz * (1.0 - cos(angle)) + ux * sin(angle);

         rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);

         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         expectedAxisAngle.setAngle(Math.PI);
         Vector3D randomVector = EuclidCoreRandomTools.nextVector3D(random);
         randomVector.normalize();
         expectedAxisAngle.setX(randomVector.getX());
         expectedAxisAngle.setY(randomVector.getY());
         expectedAxisAngle.setZ(randomVector.getZ());
         double ux = expectedAxisAngle.getX();
         double uy = expectedAxisAngle.getY();
         double uz = expectedAxisAngle.getZ();
         double angle = expectedAxisAngle.getAngle();

         // The axis angle is 'sane' and the conversion to a matrix is simple (no edge case).
         // See Wikipedia for the conversion: https://en.wikipedia.org/wiki/Rotation_matrix
         m00 = cos(angle) + ux * ux * (1.0 - cos(angle));
         m11 = cos(angle) + uy * uy * (1.0 - cos(angle));
         m22 = cos(angle) + uz * uz * (1.0 - cos(angle));

         m01 = ux * uy * (1.0 - cos(angle)) - uz * sin(angle);
         m10 = ux * uy * (1.0 - cos(angle)) + uz * sin(angle);

         m20 = ux * uz * (1.0 - cos(angle)) - uy * sin(angle);
         m02 = ux * uz * (1.0 - cos(angle)) + uy * sin(angle);

         m12 = uy * uz * (1.0 - cos(angle)) - ux * sin(angle);
         m21 = uy * uz * (1.0 - cos(angle)) + ux * sin(angle);

         rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);

         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
      }

      // Test edge cases
      // Zero rotation
      m00 = m11 = m22 = 1.0;
      m01 = m02 = m12 = 0.0;
      m10 = m20 = m21 = 0.0;
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleIsSetToZero(actualAxisAngle);

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
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(1.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI / 2.0, actualAxisAngle.getAngle(), EPSILON);

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
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(1.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

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
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI / 2.0, actualAxisAngle.getAngle(), EPSILON);

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
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

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
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI / 2.0, actualAxisAngle.getAngle(), EPSILON);

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
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(1.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      // Pi around xy (as axis-angle: (x = sqrt(2)/2, y = sqrt(2)/2, z = 0, angle = Pi)
      double sqrt2Over2 = EuclidCoreTools.squareRoot(2.0) / 2.0;
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
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(sqrt2Over2, actualAxisAngle.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getY(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

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
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(sqrt2Over2, actualAxisAngle.getX(), EPSILON);
      assertEquals(0.0, actualAxisAngle.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

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
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      assertEquals(0.0, actualAxisAngle.getX(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getY(), EPSILON);
      assertEquals(sqrt2Over2, actualAxisAngle.getZ(), EPSILON);
      assertEquals(Math.PI, actualAxisAngle.getAngle(), EPSILON);

      rotationMatrix.setUnsafe(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);
      rotationMatrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN);
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, actualAxisAngle);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(actualAxisAngle);

      // Test with a RotationScaleMatrix
      for (int i = 0; i < 1000; i++)
      {
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationScaleMatrixReadOnly rotationScaleMatrixCopy = new RotationScaleMatrix(rotationScaleMatrix);
         m00 = rotationScaleMatrix.getRotationMatrix().getM00();
         m01 = rotationScaleMatrix.getRotationMatrix().getM01();
         m02 = rotationScaleMatrix.getRotationMatrix().getM02();
         m10 = rotationScaleMatrix.getRotationMatrix().getM10();
         m11 = rotationScaleMatrix.getRotationMatrix().getM11();
         m12 = rotationScaleMatrix.getRotationMatrix().getM12();
         m20 = rotationScaleMatrix.getRotationMatrix().getM20();
         m21 = rotationScaleMatrix.getRotationMatrix().getM21();
         m22 = rotationScaleMatrix.getRotationMatrix().getM22();
         AxisAngleConversion.convertMatrixToAxisAngle(rotationScaleMatrix, actualAxisAngle);
         AxisAngleConversion.convertMatrixToAxisAngle(rotationScaleMatrix.getRotationMatrix(), expectedAxisAngle);
         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
         EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);
         // Assert the parameter does not get modified
         assertEquals(rotationScaleMatrix, rotationScaleMatrixCopy);
      }
   }

   @Test
   public void testYawPitchRollToAxisAngle() throws Exception
   {
      AxisAngle expectedAxisAngle = new AxisAngle();
      AxisAngle actualAxisAngle = new AxisAngle();
      RotationMatrix rotationMatrix = new RotationMatrix();
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;

      double deltaAngle = 0.1 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double pitch = -Math.PI / 2.0; pitch <= Math.PI / 2.0; pitch += deltaAngle)
         {
            for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
            {
               double cYaw = EuclidCoreTools.cos(yaw);
               double sYaw = EuclidCoreTools.sin(yaw);
               double cPitch = EuclidCoreTools.cos(pitch);
               double sPitch = EuclidCoreTools.sin(pitch);
               double cRoll = EuclidCoreTools.cos(roll);
               double sRoll = EuclidCoreTools.sin(roll);

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
               AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, expectedAxisAngle);
               AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, actualAxisAngle);
               EuclidCoreTestTools.assertAxisUnitary(actualAxisAngle, EPSILON);

               EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expectedAxisAngle, actualAxisAngle, EPSILON);
            }
         }
      }
   }
}
