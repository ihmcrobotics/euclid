package us.ihmc.euclid.orientation.interfaces;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public abstract class Orientation3DBasicsTest
{
   public abstract Orientation3DBasics createEmptyOrientation3DBasics();

   private Orientation3DBasics createRandomOrientation3DBasics(Random random)
   {
      Orientation3DBasics rand = createEmptyOrientation3DBasics();
      rand.set(EuclidCoreRandomTools.nextQuaternion(random));
      return rand;
   }

   public abstract double getEpsilon();

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(258097);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Orientation3DBasics orientation = createRandomOrientation3DBasics(random);
         Orientation3DBasics inverse = createEmptyOrientation3DBasics();
         inverse.set(orientation);
         inverse.invert();

         Vector3D vectorOriginal = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vectorTransformed = new Vector3D();

         Orientation3DBasics neutral = createEmptyOrientation3DBasics();
         neutral.set(orientation);
         neutral.append(inverse);

         neutral.transform(vectorOriginal, vectorTransformed);

         EuclidCoreTestTools.assertTuple3DEquals(vectorOriginal, vectorTransformed, getEpsilon());

         neutral.set(orientation);
         neutral.prepend(inverse);
         neutral.transform(vectorOriginal, vectorTransformed);
         EuclidCoreTestTools.assertTuple3DEquals(vectorOriginal, vectorTransformed, getEpsilon());
      }
   }

   @Test
   public void testSetRotationMatrix() throws Exception
   {
      Random random = new Random(43543);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double m00 = rotationMatrix.getM00();
         double m01 = rotationMatrix.getM01();
         double m02 = rotationMatrix.getM02();
         double m10 = rotationMatrix.getM10();
         double m11 = rotationMatrix.getM11();
         double m12 = rotationMatrix.getM12();
         double m20 = rotationMatrix.getM20();
         double m21 = rotationMatrix.getM21();
         double m22 = rotationMatrix.getM22();
         orientation.setRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);

         Vector3D vectorOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         rotationMatrix.transform(vectorOriginal, expected);
         orientation.transform(vectorOriginal, actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double m00 = rotationMatrix.getM00();
         double m01 = rotationMatrix.getM01();
         double m02 = rotationMatrix.getM02();
         double m10 = rotationMatrix.getM10();
         double m11 = rotationMatrix.getM11();
         double m12 = rotationMatrix.getM12();
         double m20 = rotationMatrix.getM20();
         double m21 = rotationMatrix.getM21();
         double m22 = rotationMatrix.getM22();
         orientation.setRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);

         double yprEpsilon = 10.0 * getEpsilon();
         assertEqualsDelta(rotationMatrix.getYaw(), orientation.getYaw(), yprEpsilon);
         assertEqualsDelta(rotationMatrix.getPitch(), orientation.getPitch(), yprEpsilon);
         assertEqualsDelta(rotationMatrix.getRoll(), orientation.getRoll(), yprEpsilon);
      }
   }

   @Test
   public void testSetAxisAngle() throws Exception
   {
      Random random = new Random(43543);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double x = axisAngle.getX();
         double y = axisAngle.getY();
         double z = axisAngle.getZ();
         double angle = axisAngle.getAngle();
         orientation.setAxisAngle(x, y, z, angle);

         Vector3D vectorOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         axisAngle.transform(vectorOriginal, expected);
         orientation.transform(vectorOriginal, actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double x = axisAngle.getX();
         double y = axisAngle.getY();
         double z = axisAngle.getZ();
         double angle = axisAngle.getAngle();
         orientation.setAxisAngle(x, y, z, angle);

         double yprEpsilon = 10.0 * getEpsilon();
         assertEqualsDelta(axisAngle.getYaw(), orientation.getYaw(), yprEpsilon);
         assertEqualsDelta(axisAngle.getPitch(), orientation.getPitch(), yprEpsilon);
         assertEqualsDelta(axisAngle.getRoll(), orientation.getRoll(), yprEpsilon);
      }
   }

   @Test
   public void testSetQuaternion() throws Exception
   {
      Random random = new Random(43543);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double x = quaternion.getX();
         double y = quaternion.getY();
         double z = quaternion.getZ();
         double s = quaternion.getS();
         orientation.setQuaternion(x, y, z, s);

         Vector3D vectorOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         quaternion.transform(vectorOriginal, expected);
         orientation.transform(vectorOriginal, actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double x = quaternion.getX();
         double y = quaternion.getY();
         double z = quaternion.getZ();
         double s = quaternion.getS();
         orientation.setQuaternion(x, y, z, s);

         double yprEpsilon = 10.0 * getEpsilon();
         assertEqualsDelta(quaternion.getYaw(), orientation.getYaw(), yprEpsilon);
         assertEqualsDelta(quaternion.getPitch(), orientation.getPitch(), yprEpsilon);
         assertEqualsDelta(quaternion.getRoll(), orientation.getRoll(), yprEpsilon);
      }
   }

   @Test
   public void testSetRotationVector() throws Exception
   {
      Random random = new Random(43543);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double x = rotationVector.getX();
         double y = rotationVector.getY();
         double z = rotationVector.getZ();
         orientation.setRotationVector(x, y, z);

         Vector3D vectorOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         new Quaternion(rotationVector).transform(vectorOriginal, expected);
         orientation.transform(vectorOriginal, actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double x = rotationVector.getX();
         double y = rotationVector.getY();
         double z = rotationVector.getZ();
         orientation.setRotationVector(x, y, z);

         double yprEpsilon = 10.0 * getEpsilon();
         assertEqualsDelta(new RotationMatrix(rotationVector).getYaw(), orientation.getYaw(), yprEpsilon);
         assertEqualsDelta(new Quaternion(rotationVector).getPitch(), orientation.getPitch(), yprEpsilon);
         assertEqualsDelta(new AxisAngle(rotationVector).getRoll(), orientation.getRoll(), yprEpsilon);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         orientation.setRotationVector(rotationVector);

         double yprEpsilon = 10.0 * getEpsilon();
         assertEqualsDelta(new RotationMatrix(rotationVector).getYaw(), orientation.getYaw(), yprEpsilon);
         assertEqualsDelta(new Quaternion(rotationVector).getPitch(), orientation.getPitch(), yprEpsilon);
         assertEqualsDelta(new AxisAngle(rotationVector).getRoll(), orientation.getRoll(), yprEpsilon);
      }
   }

   @Test
   public void testSetYawPitchRoll() throws Exception
   {
      Random random = new Random(43543);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double yaw = yawPitchRoll[0];
         double pitch = yawPitchRoll[1];
         double roll = yawPitchRoll[2];
         orientation.setYawPitchRoll(yaw, pitch, roll);

         Vector3D vectorOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         RotationMatrixTools.applyRollRotation(roll, vectorOriginal, expected);
         RotationMatrixTools.applyPitchRotation(pitch, expected, expected);
         RotationMatrixTools.applyYawRotation(yaw, expected, expected);
         orientation.transform(vectorOriginal, actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double yaw = yawPitchRoll[0];
         double pitch = yawPitchRoll[1];
         double roll = yawPitchRoll[2];
         orientation.setYawPitchRoll(yaw, pitch, roll);

         double yprEpsilon = 10.0 * getEpsilon();
         assertEqualsDelta(yawPitchRoll[0], orientation.getYaw(), yprEpsilon);
         assertEqualsDelta(yawPitchRoll[1], orientation.getPitch(), yprEpsilon);
         assertEqualsDelta(yawPitchRoll[2], orientation.getRoll(), yprEpsilon);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         orientation.setYawPitchRoll(yawPitchRoll);

         double yprEpsilon = 10.0 * getEpsilon();
         assertEqualsDelta(yawPitchRoll[0], orientation.getYaw(), yprEpsilon);
         assertEqualsDelta(yawPitchRoll[1], orientation.getPitch(), yprEpsilon);
         assertEqualsDelta(yawPitchRoll[2], orientation.getRoll(), yprEpsilon);
      }
   }

   @Test
   public void testSetEuler() throws Exception
   {
      Random random = new Random(43543);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D eulerAngles = new Vector3D(EuclidCoreRandomTools.nextYawPitchRollArray(random));
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double rotX = eulerAngles.getX();
         double rotY = eulerAngles.getY();
         double rotZ = eulerAngles.getZ();
         orientation.setEuler(rotX, rotY, rotZ);

         Vector3D vectorOriginal = EuclidCoreRandomTools.nextVector3D(random);

         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         RotationMatrixTools.applyRollRotation(rotX, vectorOriginal, expected);
         RotationMatrixTools.applyPitchRotation(rotY, expected, expected);
         RotationMatrixTools.applyYawRotation(rotZ, expected, expected);
         orientation.transform(vectorOriginal, actual);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D eulerAngles = new Vector3D(EuclidCoreRandomTools.nextYawPitchRollArray(random));
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         double rotX = eulerAngles.getX();
         double rotY = eulerAngles.getY();
         double rotZ = eulerAngles.getZ();
         orientation.setEuler(rotX, rotY, rotZ);

         double yprEpsilon = 10.0 * getEpsilon();
         assertEqualsDelta(rotZ, orientation.getYaw(), yprEpsilon);
         assertEqualsDelta(rotY, orientation.getPitch(), yprEpsilon);
         assertEqualsDelta(rotX, orientation.getRoll(), yprEpsilon);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D eulerAngles = new Vector3D(EuclidCoreRandomTools.nextYawPitchRollArray(random));
         Orientation3DBasics orientation = createEmptyOrientation3DBasics();
         orientation.setEuler(eulerAngles);

         double yprEpsilon = 10.0 * getEpsilon();
         assertEqualsDelta(eulerAngles.getZ(), orientation.getYaw(), yprEpsilon);
         assertEqualsDelta(eulerAngles.getY(), orientation.getPitch(), yprEpsilon);
         assertEqualsDelta(eulerAngles.getX(), orientation.getRoll(), yprEpsilon);
      }
   }

   @Test
   public void testSetAndInvert() throws Exception
   {
      Random random = new Random(258097);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Orientation3DBasics orientation = createRandomOrientation3DBasics(random);
         Orientation3DBasics inverse = createEmptyOrientation3DBasics();
         inverse.setAndInvert(orientation);

         Vector3D vectorOriginal = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vectorTransformed = new Vector3D();

         Orientation3DBasics neutral = createEmptyOrientation3DBasics();
         neutral.set(orientation);
         neutral.append(inverse);

         neutral.transform(vectorOriginal, vectorTransformed);

         EuclidCoreTestTools.assertTuple3DEquals(vectorOriginal, vectorTransformed, getEpsilon());

         neutral.set(orientation);
         neutral.prepend(inverse);
         neutral.transform(vectorOriginal, vectorTransformed);
         EuclidCoreTestTools.assertTuple3DEquals(vectorOriginal, vectorTransformed, getEpsilon());
      }
   }

   @Test
   public void testAppend() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly appended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.multiply(appended);
         actualOrientation.append(appended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly appended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.multiply(appended);
         actualOrientation.append(appended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly appended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.multiply(appended);
         actualOrientation.append(appended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendInvertThis() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly appended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.multiplyConjugateThis(appended);
         actualOrientation.appendInvertThis(appended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly appended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.multiplyInvertThis(appended);
         actualOrientation.appendInvertThis(appended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly appended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.multiplyTransposeThis(appended);
         actualOrientation.appendInvertThis(appended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendInvertOther() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly appended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.multiplyConjugateOther(appended);
         actualOrientation.appendInvertOther(appended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly appended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.multiplyInvertOther(appended);
         actualOrientation.appendInvertOther(appended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly appended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.multiplyTransposeOther(appended);
         actualOrientation.appendInvertOther(appended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendInvertBoth() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly appended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.multiplyConjugateBoth(appended);
         actualOrientation.appendInvertBoth(appended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly appended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.multiplyInvertBoth(appended);
         actualOrientation.appendInvertBoth(appended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly appended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.multiplyTransposeBoth(appended);
         actualOrientation.appendInvertBoth(appended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrepend() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly prepended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.preMultiply(prepended);
         actualOrientation.prepend(prepended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly prepended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.preMultiply(prepended);
         actualOrientation.prepend(prepended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly prepended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.preMultiply(prepended);
         actualOrientation.prepend(prepended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependInvertThis() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly prepended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.preMultiplyConjugateThis(prepended);
         actualOrientation.prependInvertThis(prepended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly prepended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.preMultiplyInvertThis(prepended);
         actualOrientation.prependInvertThis(prepended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly prepended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.preMultiplyTransposeThis(prepended);
         actualOrientation.prependInvertThis(prepended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependInvertOther() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly prepended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.preMultiplyConjugateOther(prepended);
         actualOrientation.prependInvertOther(prepended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly prepended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.preMultiplyInvertOther(prepended);
         actualOrientation.prependInvertOther(prepended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly prepended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.preMultiplyTransposeOther(prepended);
         actualOrientation.prependInvertOther(prepended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependInvertBoth() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly prepended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.preMultiplyConjugateBoth(prepended);
         actualOrientation.prependInvertBoth(prepended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly prepended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.preMultiplyInvertBoth(prepended);
         actualOrientation.prependInvertBoth(prepended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly prepended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.preMultiplyTransposeBoth(prepended);
         actualOrientation.prependInvertBoth(prepended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

}
