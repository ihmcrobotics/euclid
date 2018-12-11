package us.ihmc.euclid.rotationConversion;

import static java.lang.Math.*;
import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class RotationVectorConversionTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testAxisAngleToRotationVector() throws Exception
   {
      Random random = new Random(4591L);
      double minMaxAngleRange = 2.0 * Math.PI;

      for (int i = 0; i < ITERATIONS; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
         Vector3D rotationVector = new Vector3D();
         double ux = axisAngle.getX();
         double uy = axisAngle.getY();
         double uz = axisAngle.getZ();
         double angle = axisAngle.getAngle();
         RotationVectorConversion.convertAxisAngleToRotationVectorImpl(ux, uy, uz, angle, rotationVector);

         assertEqualsDelta(rotationVector.length(), Math.abs(angle), EPSILON);
         assertEqualsDelta(rotationVector.getX(), angle * ux, EPSILON);
         assertEqualsDelta(rotationVector.getY(), angle * uy, EPSILON);
         assertEqualsDelta(rotationVector.getZ(), angle * uz, EPSILON);
      }

      // Test with a non-unitary axis.
      Vector3D expectedRotationVector = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D actualRotationVector = new Vector3D();

      double ux = expectedRotationVector.getX();
      double uy = expectedRotationVector.getY();
      double uz = expectedRotationVector.getZ();
      double angle = expectedRotationVector.length();
      RotationVectorConversion.convertAxisAngleToRotationVectorImpl(ux, uy, uz, angle, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);

      RotationVectorConversion.convertAxisAngleToRotationVectorImpl(0.0, 0.0, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(actualRotationVector);

      RotationVectorConversion.convertAxisAngleToRotationVectorImpl(Double.NaN, 0.0, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertAxisAngleToRotationVectorImpl(0.0, Double.NaN, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertAxisAngleToRotationVectorImpl(0.0, 0.0, Double.NaN, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertAxisAngleToRotationVectorImpl(0.0, 0.0, 0.0, Double.NaN, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      for (int i = 0; i < 100; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
         AxisAngle axisAngleCopy = new AxisAngle(axisAngle);
         ux = axisAngle.getX();
         uy = axisAngle.getY();
         uz = axisAngle.getZ();
         angle = axisAngle.getAngle();
         RotationVectorConversion.convertAxisAngleToRotationVectorImpl(ux, uy, uz, angle, expectedRotationVector);
         RotationVectorConversion.convertAxisAngleToRotationVector(axisAngle, actualRotationVector);
         EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);
         // Assert the parameter that does get modified
         assertTrue(axisAngle.equals(axisAngleCopy));
      }
   }

   @Test
   public void testQuaternionToRotationVector() throws Exception
   {
      Random random = new Random(1641L);
      double minMaxAngleRange = 2.0 * Math.PI;
      Vector3D expectedRotationVector = new Vector3D();
      Vector3D actualRotationVector = new Vector3D();
      Quaternion quaternion = new Quaternion();

      for (int i = 0; i < 10000; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
         double ux = axisAngle.getX();
         double uy = axisAngle.getY();
         double uz = axisAngle.getZ();
         double angle = axisAngle.getAngle();
         expectedRotationVector.setX(ux * angle);
         expectedRotationVector.setY(uy * angle);
         expectedRotationVector.setZ(uz * angle);

         double qs = Math.cos(angle / 2.0);
         double qx = ux * Math.sin(angle / 2.0);
         double qy = uy * Math.sin(angle / 2.0);
         double qz = uz * Math.sin(angle / 2.0);

         quaternion.setUnsafe(qx, qy, qz, qs);
         RotationVectorConversion.convertQuaternionToRotationVector(quaternion, actualRotationVector);
         EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);
      }

      // test some very small angles
      minMaxAngleRange = 1.0e-8;
      for (int i = 0; i < 10000; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
         double ux = axisAngle.getX();
         double uy = axisAngle.getY();
         double uz = axisAngle.getZ();
         double angle = axisAngle.getAngle();
         expectedRotationVector.setX(ux * angle);
         expectedRotationVector.setY(uy * angle);
         expectedRotationVector.setZ(uz * angle);

         double qs = Math.cos(angle / 2.0);
         double qx = ux * Math.sin(angle / 2.0);
         double qy = uy * Math.sin(angle / 2.0);
         double qz = uz * Math.sin(angle / 2.0);

         quaternion.setUnsafe(qx, qy, qz, qs);
         RotationVectorConversion.convertQuaternionToRotationVector(quaternion, actualRotationVector);
         EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);
      }

      quaternion.setUnsafe(0.0, 0.0, 0.0, 0.0);
      RotationVectorConversion.convertQuaternionToRotationVector(quaternion, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(actualRotationVector);

      quaternion.setUnsafe(Double.NaN, 0.0, 0.0, 0.0);
      RotationVectorConversion.convertQuaternionToRotationVector(quaternion, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      quaternion.setUnsafe(0.0, Double.NaN, 0.0, 0.0);
      RotationVectorConversion.convertQuaternionToRotationVector(quaternion, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      quaternion.setUnsafe(0.0, 0.0, Double.NaN, 0.0);
      RotationVectorConversion.convertQuaternionToRotationVector(quaternion, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      quaternion.setUnsafe(0.0, 0.0, 0.0, Double.NaN);
      RotationVectorConversion.convertQuaternionToRotationVector(quaternion, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);
   }

   @Test
   public void testMatrixToRotationVector() throws Exception
   {
      Random random = new Random(3651651L);
      double minMaxAngleRange = Math.PI;
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;
      Vector3D expectedRotationVector = new Vector3D();
      Vector3D actualRotationVector = new Vector3D();

      for (int i = 0; i < 10000; i++)
      {
         AxisAngle axisAngle = new AxisAngle();
         axisAngle.setAngle(Math.PI);
         Vector3D randomVector = EuclidCoreRandomTools.nextVector3D(random);
         randomVector.normalize();
         axisAngle.setX(randomVector.getX());
         axisAngle.setY(randomVector.getY());
         axisAngle.setZ(randomVector.getZ());
         double ux = axisAngle.getX();
         double uy = axisAngle.getY();
         double uz = axisAngle.getZ();
         double angle = axisAngle.getAngle();
         expectedRotationVector.setX(ux * angle);
         expectedRotationVector.setY(uy * angle);
         expectedRotationVector.setZ(uz * angle);

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

         RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);

         if (actualRotationVector.getX() * expectedRotationVector.getX() < 0.0)
         {
            actualRotationVector.negate();
         }
         EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);
      }

      for (int i = 0; i < 10000; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random, minMaxAngleRange);
         double ux = axisAngle.getX();
         double uy = axisAngle.getY();
         double uz = axisAngle.getZ();
         double angle = axisAngle.getAngle();
         expectedRotationVector.setX(ux * angle);
         expectedRotationVector.setY(uy * angle);
         expectedRotationVector.setZ(uz * angle);

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

         RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
         EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);
      }

      // Test edge cases
      // Zero rotation
      m00 = m11 = m22 = 1.0;
      m01 = m02 = m12 = 0.0;
      m10 = m20 = m21 = 0.0;
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(actualRotationVector);

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
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      assertEqualsDelta(Math.PI / 2.0, actualRotationVector.getX(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getY(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getZ(), EPSILON);

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
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      assertEqualsDelta(Math.PI, actualRotationVector.getX(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getY(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getZ(), EPSILON);

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
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      assertEqualsDelta(0.0, actualRotationVector.getX(), EPSILON);
      assertEqualsDelta(Math.PI / 2.0, actualRotationVector.getY(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getZ(), EPSILON);

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
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      assertEqualsDelta(0.0, actualRotationVector.getX(), EPSILON);
      assertEqualsDelta(Math.PI, actualRotationVector.getY(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getZ(), EPSILON);

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
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      assertEqualsDelta(0.0, actualRotationVector.getX(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getY(), EPSILON);
      assertEqualsDelta(Math.PI / 2.0, actualRotationVector.getZ(), EPSILON);

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
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      assertEqualsDelta(0.0, actualRotationVector.getX(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getY(), EPSILON);
      assertEqualsDelta(Math.PI, actualRotationVector.getZ(), EPSILON);

      // Pi around xy (as axis-angle: (x = sqrt(2)/2, y = sqrt(2)/2, z = 0, angle = Pi)
      double sqrt2Over2 = Math.sqrt(2.0) / 2.0;
      m00 = 0.0;
      m01 = 1.0;
      m02 = 0.0;
      m10 = 1.0;
      m11 = 0.0;
      m12 = 0.0;
      m20 = 0.0;
      m21 = 0.0;
      m22 = -1.0;
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      assertEqualsDelta(Math.PI * sqrt2Over2, actualRotationVector.getX(), EPSILON);
      assertEqualsDelta(Math.PI * sqrt2Over2, actualRotationVector.getY(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getZ(), EPSILON);

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
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      assertEqualsDelta(Math.PI * sqrt2Over2, actualRotationVector.getX(), EPSILON);
      assertEqualsDelta(0.0, actualRotationVector.getY(), EPSILON);
      assertEqualsDelta(Math.PI * sqrt2Over2, actualRotationVector.getZ(), EPSILON);

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
      RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, actualRotationVector);
      assertEqualsDelta(0.0, actualRotationVector.getX(), EPSILON);
      assertEqualsDelta(Math.PI * sqrt2Over2, actualRotationVector.getY(), EPSILON);
      assertEqualsDelta(Math.PI * sqrt2Over2, actualRotationVector.getZ(), EPSILON);

      RotationVectorConversion.convertMatrixToRotationVectorImpl(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertMatrixToRotationVectorImpl(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertMatrixToRotationVectorImpl(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertMatrixToRotationVectorImpl(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertMatrixToRotationVectorImpl(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertMatrixToRotationVectorImpl(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertMatrixToRotationVectorImpl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertMatrixToRotationVectorImpl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      RotationVectorConversion.convertMatrixToRotationVectorImpl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, actualRotationVector);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualRotationVector);

      // Test with an actual matrix
      for (int i = 0; i < 1000; i++)
      {
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix rotationMatrixCopy = new RotationMatrix(rotationMatrix);
         m00 = rotationMatrix.getM00();
         m01 = rotationMatrix.getM01();
         m02 = rotationMatrix.getM02();
         m10 = rotationMatrix.getM10();
         m11 = rotationMatrix.getM11();
         m12 = rotationMatrix.getM12();
         m20 = rotationMatrix.getM20();
         m21 = rotationMatrix.getM21();
         m22 = rotationMatrix.getM22();
         RotationVectorConversion.convertMatrixToRotationVector(rotationMatrix, actualRotationVector);
         RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, expectedRotationVector);
         EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);
         // Assert the parameter that does get modified
         assertTrue(rotationMatrix.equals(rotationMatrixCopy));
      }

      // Test with a RotationScaleMatrix
      for (int i = 0; i < 1000; i++)
      {
         RotationScaleMatrix rotationScaleMatrix = EuclidCoreRandomTools.nextRotationScaleMatrix(random, 10.0);
         RotationScaleMatrix rotationScaleMatrixCopy = new RotationScaleMatrix(rotationScaleMatrix);
         m00 = rotationScaleMatrix.getRotationMatrix().getM00();
         m01 = rotationScaleMatrix.getRotationMatrix().getM01();
         m02 = rotationScaleMatrix.getRotationMatrix().getM02();
         m10 = rotationScaleMatrix.getRotationMatrix().getM10();
         m11 = rotationScaleMatrix.getRotationMatrix().getM11();
         m12 = rotationScaleMatrix.getRotationMatrix().getM12();
         m20 = rotationScaleMatrix.getRotationMatrix().getM20();
         m21 = rotationScaleMatrix.getRotationMatrix().getM21();
         m22 = rotationScaleMatrix.getRotationMatrix().getM22();
         RotationVectorConversion.convertMatrixToRotationVector(rotationScaleMatrix, actualRotationVector);
         RotationVectorConversion.convertMatrixToRotationVectorImpl(m00, m01, m02, m10, m11, m12, m20, m21, m22, expectedRotationVector);
         EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);
         // Assert the parameter that does get modified
         assertTrue(rotationScaleMatrix.equals(rotationScaleMatrixCopy));
      }
   }

   @Test
   public void testYawPitchRollToRotationVector() throws Exception
   {
      Vector3D expectedRotationVector = new Vector3D();
      Vector3D actualRotationVector = new Vector3D();
      double[] yawPitchRoll = new double[3];

      double deltaAngle = 0.1 * Math.PI;

      for (double yaw = -Math.PI; yaw <= Math.PI; yaw += deltaAngle)
      {
         for (double roll = -Math.PI; roll <= Math.PI; roll += deltaAngle)
         {
            for (double pitch = -Math.PI / 2.0; pitch <= Math.PI / 2.0; pitch += deltaAngle)
            {
               Quaternion quaternion = new Quaternion();
               // Trust the conversion to quaternion is well tested
               QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, quaternion);
               RotationVectorConversion.convertQuaternionToRotationVector(quaternion, expectedRotationVector);
               RotationVectorConversion.convertYawPitchRollToRotationVector(yaw, pitch, roll, actualRotationVector);
               EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);

               yawPitchRoll = new double[] {yaw, pitch, roll};
               RotationVectorConversion.convertYawPitchRollToRotationVector(yawPitchRoll, actualRotationVector);
               EuclidCoreTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, EPSILON);
               assertTrue(yawPitchRoll[0] == yaw);
               assertTrue(yawPitchRoll[1] == pitch);
               assertTrue(yawPitchRoll[2] == roll);
            }
         }
      }
   }
}
