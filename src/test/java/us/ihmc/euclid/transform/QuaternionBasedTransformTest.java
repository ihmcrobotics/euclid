package us.ihmc.euclid.transform;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class QuaternionBasedTransformTest extends TransformTest<QuaternionBasedTransform>
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(345L);

      { // Test empty constructor
         QuaternionBasedTransform transform = new QuaternionBasedTransform();
         EuclidCoreTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslationVector());
      }

      { // Test QuaternionBasedTransform(QuaternionBasedTransform other)
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = new QuaternionBasedTransform(expected);
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      }

      { // Test QuaternionBasedTransform(RigidBodyTransform rigidBodyTransform)
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = new QuaternionBasedTransform(new RigidBodyTransform(expected));
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      }

      { // Test QuaternionBasedTransform(DenseMatrix64F matrix)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7, 1);
         quaternion.get(denseMatrix);
         translation.get(4, denseMatrix);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(denseMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(double[] array)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double[] array = new double[7];
         quaternion.get(array);
         translation.get(4, array);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(array);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(RotationMatrix rotationMatrix, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(new RotationMatrix(quaternion), translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(QuaternionReadOnly quaternion, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(quaternion, translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(AxisAngleReadOnly axisAngle, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(new AxisAngle(quaternion), translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

      assertNotEquals(transform.getQuaternion().getX(), 0.0);
      assertNotEquals(transform.getQuaternion().getY(), 0.0);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0);
      assertNotEquals(transform.getQuaternion().getS(), 1.0);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0);

      transform.setToZero();

      EuclidCoreTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslationVector());
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

      assertFalse(Double.isNaN(transform.getQuaternion().getX()));
      assertFalse(Double.isNaN(transform.getQuaternion().getY()));
      assertFalse(Double.isNaN(transform.getQuaternion().getZ()));
      assertFalse(Double.isNaN(transform.getQuaternion().getS()));
      assertFalse(Double.isNaN(transform.getTranslationVector().getX()));
      assertFalse(Double.isNaN(transform.getTranslationVector().getY()));
      assertFalse(Double.isNaN(transform.getTranslationVector().getZ()));

      transform.setToNaN();

      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(transform.getTranslationVector());

      transform.setToZero();
      EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), transform.getQuaternion(), EPS);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), transform.getTranslationVector(), EPS);

      transform.setRotationToNaN();
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), transform.getTranslationVector(), EPS);

      transform.setToZero();
      transform.setTranslationToNaN();
      EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), transform.getQuaternion(), EPS);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(transform.getTranslationVector());
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      QuaternionBasedTransform transform = new QuaternionBasedTransform();

      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertFalse(transform.containsNaN());
      transform.setUnsafe(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN);
      assertTrue(transform.containsNaN());
   }

   @Test
   public void testResetRotation() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      Vector3D expectedTranslation = new Vector3D();
      transform.getTranslation(expectedTranslation);

      assertNotEquals(transform.getQuaternion().getX(), 0.0);
      assertNotEquals(transform.getQuaternion().getY(), 0.0);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0);
      assertNotEquals(transform.getQuaternion().getS(), 1.0);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0);

      transform.setRotationToZero();

      EuclidCoreTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, transform.getTranslationVector(), EPS);
   }

   @Test
   public void testResetTranslation() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      Quaternion expectedQuaternion = new Quaternion();
      transform.getRotation(expectedQuaternion);

      assertNotEquals(transform.getQuaternion().getX(), 0.0);
      assertNotEquals(transform.getQuaternion().getY(), 0.0);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0);
      assertNotEquals(transform.getQuaternion().getS(), 1.0);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0);

      transform.setTranslationToZero();

      EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, transform.getQuaternion(), EPS);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslationVector());
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform actualTransform = new QuaternionBasedTransform();

      { // Test set(double qx, double qy, double qz, double qs, double x, double y, double z)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double qx = quaternion.getX();
         double qy = quaternion.getY();
         double qz = quaternion.getZ();
         double qs = quaternion.getS();
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.set(qx, qy, qz, qs, x, y, z);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setUnsafe(double qx, double qy, double qz, double qs, double x, double y, double z)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double qx = quaternion.getX();
         double qy = quaternion.getY();
         double qz = quaternion.getZ();
         double qs = quaternion.getS();
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.setUnsafe(qx, qy, qz, qs, x, y, z);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(QuaternionBasedTransform other)
         QuaternionBasedTransform expectedTransform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         actualTransform.set(expectedTransform);
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expectedTransform, actualTransform, EPS);
      }

      { // Test set(RigidBodyTransform rigidBodyTransform)
         QuaternionBasedTransform expectedTransform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         actualTransform.set(new RigidBodyTransform(expectedTransform));
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expectedTransform, actualTransform, EPS);
      }

      { // Test set(DenseMatrix64F matrix)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7, 1);
         quaternion.get(denseMatrix);
         translation.get(4, denseMatrix);
         actualTransform.set(denseMatrix);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(double[] array)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double[] array = new double[7];
         quaternion.get(array);
         translation.get(4, array);
         actualTransform.set(array);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(RotationMatrix rotationMatrix, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         actualTransform.set(new RotationMatrix(quaternion), translation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(AxisAngleReadOnly axisAngle, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         actualTransform.set(new AxisAngle(quaternion), translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(QuaternionReadOnly quaternion, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         actualTransform.set(quaternion, translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testSetIdentity() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

      assertNotEquals(transform.getQuaternion().getX(), 0.0);
      assertNotEquals(transform.getQuaternion().getY(), 0.0);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0);
      assertNotEquals(transform.getQuaternion().getS(), 1.0);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0);

      transform.setIdentity();

      EuclidCoreTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslationVector());
   }

   @Test
   public void testSetRotation() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform actualTransform = new QuaternionBasedTransform();
      Vector3D expectedTranslation = new Vector3D();
      actualTransform.getTranslation(expectedTranslation);

      { // Test setRotation(AxisAngleReadOnly axisAngle)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         actualTransform.setRotation(new AxisAngle(quaternion));
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotation(QuaternionReadOnly quaternion)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         actualTransform.setRotation(quaternion);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotation(RotationMatrix rotationMatrix)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         actualTransform.setRotation(new RotationMatrix(quaternion));
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotation(VectorReadOnly rotationVector)
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D rotationVector = new Vector3D();
         quaternion.getRotationVector(rotationVector);
         actualTransform.setRotation(rotationVector);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotationYaw(double yaw)
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         rigidBodyTransform.setRotationYaw(yaw);
         QuaternionBasedTransform expectedTransform = new QuaternionBasedTransform(rigidBodyTransform);
         actualTransform.setRotationYaw(yaw);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expectedTransform, actualTransform, EPS);
      }

      { // Test setRotationPitch(double pitch)
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         rigidBodyTransform.setRotationPitch(pitch);
         QuaternionBasedTransform expectedTransform = new QuaternionBasedTransform(rigidBodyTransform);
         actualTransform.setRotationPitch(pitch);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expectedTransform, actualTransform, EPS);
      }

      { // Test setRotationRoll(double roll)
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         rigidBodyTransform.setRotationRoll(roll);
         QuaternionBasedTransform expectedTransform = new QuaternionBasedTransform(rigidBodyTransform);
         actualTransform.setRotationRoll(roll);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expectedTransform, actualTransform, EPS);
      }

      { // Test setRotationYawPitchRoll(double roll)
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         rigidBodyTransform.setRotationYawPitchRoll(yaw, pitch, roll);
         QuaternionBasedTransform expectedTransform = new QuaternionBasedTransform(rigidBodyTransform);
         actualTransform.setRotationYawPitchRoll(yaw, pitch, roll);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expectedTransform, actualTransform, EPS);

         actualTransform.setRotationToZero();
         actualTransform.setRotationYawPitchRoll(new double[] {yaw, pitch, roll});
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expectedTransform, actualTransform, EPS);

         actualTransform.setRotationToZero();
         actualTransform.setRotationEuler(new Vector3D(roll, pitch, yaw));
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expectedTransform, actualTransform, EPS);

         actualTransform.setRotationToZero();
         actualTransform.setRotationEuler(roll, pitch, yaw);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expectedTransform, actualTransform, EPS);
      }
   }

   @Test
   public void testSetTranslation() throws Exception
   {
      Random random = new Random(456456L);
      QuaternionBasedTransform actualTransform = new QuaternionBasedTransform();
      Quaternion expectedQuaternion = new Quaternion();
      actualTransform.getRotation(expectedQuaternion);

      { // Test individual setTranslation(X/Y/Z)(double)
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.setTranslationX(x);
         actualTransform.setTranslationY(y);
         actualTransform.setTranslationZ(z);
         for (int row = 0; row < 3; row++)
         {
            EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualTransform.getQuaternion(), EPS);
            EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
         }
      }

      { // Test setTranslation(double x, double y, double z)
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.setTranslation(x, y, z);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setTranslation(TupleReadOnly translation)
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         actualTransform.setTranslation(translation);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testGet() throws Exception
   {
      Random random = new Random(34543L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      QuaternionReadOnly expectedQuaternion = transform.getQuaternion();
      Vector3DReadOnly expectedTranslation = transform.getTranslationVector();

      { // Test get(DenseMatrix64F matrixToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7, 1);
         transform.get(denseMatrix);
         actualQuaternion.set(denseMatrix);
         actualTranslation.set(4, denseMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(DenseMatrix64F matrixToPack, int startRow, int column)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         int startRow = random.nextInt(10);
         int column = random.nextInt(10);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7 + startRow, 1 + column);
         transform.get(startRow, column, denseMatrix);
         actualQuaternion.set(startRow, column, denseMatrix);
         actualTranslation.set(4 + startRow, column, denseMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(double[] transformArrayToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         double[] denseMatrix = new double[7];
         transform.get(denseMatrix);
         actualQuaternion.set(denseMatrix);
         actualTranslation.set(4, denseMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(QuaternionBasics quaternionToPack, TupleBasics translationToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         transform.get(actualQuaternion, actualTranslation);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(RotationMatrix rotationMarixToPack, TupleBasics translationToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         RotationMatrix rotationMatrix = new RotationMatrix();
         transform.get(rotationMatrix, actualTranslation);
         actualQuaternion.set(rotationMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(RotationScaleMatrix rotationMarixToPack, TupleBasics translationToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
         transform.get(rotationScaleMatrix, actualTranslation);
         rotationScaleMatrix.getRotation(actualQuaternion);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }
   }

   @Test
   public void testGetRotation() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      QuaternionReadOnly expectedQuaternion = transform.getQuaternion();
      Quaternion actualQuaternion = new Quaternion();

      { // Test getRotation(RotationMatrix rotationMatrixToPack)
         RotationMatrix rotationMatrix = new RotationMatrix();
         transform.getRotation(rotationMatrix);
         actualQuaternion.set(rotationMatrix);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(QuaternionBasics quaternionToPack)
         actualQuaternion.setToNaN();
         transform.getRotation(actualQuaternion);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(AxisAngleBasics axisAngleToPack)
         AxisAngle axisAngle = new AxisAngle();
         transform.getRotation(axisAngle);
         actualQuaternion.set(axisAngle);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(Vector3DBasics rotationVectorToPack)
         Vector3D rotationVector = new Vector3D();
         transform.getRotation(rotationVector);
         actualQuaternion.setRotationVector(rotationVector);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotationYawPitchRoll(double[] yawPitchRollToPack)
         double[] yawPitchRoll = new double[3];
         transform.getRotationYawPitchRoll(yawPitchRoll);
         actualQuaternion.setYawPitchRoll(yawPitchRoll);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(Vector3DBasics rotationVectorToPack)
         Vector3D eulerAngles = new Vector3D();
         transform.getRotationEuler(eulerAngles);
         actualQuaternion.setEuler(eulerAngles);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedQuaternion, actualQuaternion, EPS);
      }
   }

   @Test
   public void testGetTranslation() throws Exception
   {
      Random random = new Random(34L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D actual = new Vector3D();

      transform.setTranslation(expected);
      transform.getTranslation(actual);

      EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      EuclidCoreTestTools.assertTuple3DEquals(transform.getTranslationVector(), actual, EPS);

      Vector3D translation = new Vector3D();
      translation.set(transform.getTranslationX(), transform.getTranslationY(), transform.getTranslationZ());
      EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
   }

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(345L);
      QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      rigidBodyTransform.set(original);
      rigidBodyTransform.invert();
      expected.set(rigidBodyTransform);

      actual.set(original);
      actual.invert();

      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

      rigidBodyTransform.set(original);
      rigidBodyTransform.invertRotation();
      expected.set(rigidBodyTransform);

      actual.set(original);
      actual.invertRotation();

      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(23542342L);

      QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      QuaternionBasedTransform q0 = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      QuaternionBasedTransform qf = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

      actual.interpolate(q0, qf, 0.0);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(q0, actual, EPS);
      actual.interpolate(qf, 0.0);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(q0, actual, EPS);
      actual.interpolate(qf, 1.0);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(qf, actual, EPS);

      actual.interpolate(q0, qf, 1.0);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(qf, actual, EPS);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double alpha = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Vector3D interpolatedVector = new Vector3D();
         Quaternion interpolatedRotation = new Quaternion();

         interpolatedVector.interpolate(q0.getTranslationVector(), qf.getTranslationVector(), alpha);
         interpolatedRotation.interpolate(q0.getQuaternion(), qf.getQuaternion(), alpha);

         expected.set(interpolatedRotation, interpolatedVector);
         actual.interpolate(q0, qf, alpha);
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

         actual.set(q0);
         actual.interpolate(qf, alpha);
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testAppendTranslation() throws Exception
   {
      Random random = new Random(35454L);

      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // appendTranslation(double x, double y, double z)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform translationTransform = new QuaternionBasedTransform();
         double x = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double z = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         translationTransform.setTranslation(x, y, z);
         expected.set(original);
         expected.multiply(translationTransform);

         actual.set(original);
         actual.appendTranslation(x, y, z);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendTranslation(Tuple3DReadOnly translation)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform translationTransform = new QuaternionBasedTransform();
         Tuple3DReadOnly translation = EuclidCoreRandomTools.nextPoint3D(random, 10.0, 10.0, 10.0);
         translationTransform.setTranslation(translation);
         expected.set(original);
         expected.multiply(translationTransform);

         actual.set(original);
         actual.appendTranslation(translation);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testAppendYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);

      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // appendYawRotation(double yaw)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getQuaternion());
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.appendYawRotation(yaw);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.appendYawRotation(yaw);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendPitchRotation(double pitch)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getQuaternion());
         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.appendPitchRotation(pitch);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.appendPitchRotation(pitch);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // appendRollRotation(double roll)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getQuaternion());
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         expectedRotation.appendRollRotation(roll);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.appendRollRotation(roll);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPrependTranslation() throws Exception
   {
      Random random = new Random(35454L);

      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // prependTranslation(double x, double y, double z)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform translationTransform = new QuaternionBasedTransform();
         double x = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double z = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double y = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         translationTransform.setTranslation(x, y, z);
         expected.set(original);
         expected.preMultiply(translationTransform);

         actual.set(original);
         actual.prependTranslation(x, y, z);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependTranslation(Tuple3DReadOnly translation)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform translationTransform = new QuaternionBasedTransform();
         Tuple3DReadOnly translation = EuclidCoreRandomTools.nextPoint3D(random, 10.0, 10.0, 10.0);
         translationTransform.setTranslation(translation);
         expected.set(original);
         expected.preMultiply(translationTransform);

         actual.set(original);
         actual.prependTranslation(translation);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPrependYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);

      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      for (int i = 0; i < ITERATIONS; i++)
      { // prependYawRotation(double yaw)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform yawTransform = new QuaternionBasedTransform();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         yawTransform.setRotationYaw(yaw);
         expected.set(original);
         expected.preMultiply(yawTransform);

         actual.set(original);
         actual.prependYawRotation(yaw);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependPitchRotation(double pitch)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform pitchTransform = new QuaternionBasedTransform();

         double pitch = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         pitchTransform.setRotationPitch(pitch);
         expected.set(original);
         expected.preMultiply(pitchTransform);

         actual.set(original);
         actual.prependPitchRotation(pitch);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // prependRollRotation(double roll)
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform rollTransform = new QuaternionBasedTransform();

         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         rollTransform.setRotationRoll(roll);
         expected.set(original);
         expected.preMultiply(rollTransform);

         actual.set(original);
         actual.prependRollRotation(roll);

         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThis() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOther() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiply() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThis() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOther() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.nextAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformGeometricallyEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransform() throws Exception
   {
      Random random = new Random(34L);
      QuaternionBasedTransform qTransform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      RigidBodyTransform rTransform = new RigidBodyTransform(qTransform);

      RigidBodyTransform rTransform2D = new RigidBodyTransform();
      rTransform2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      rTransform2D.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
      QuaternionBasedTransform qTransform2D = new QuaternionBasedTransform(rTransform2D);

      { // Test transform(PointBasics pointToTransform)
         Point3D original = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test transform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         Point3D original = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test transform(VectorBasics vectorToTransform)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test transform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test transform(QuaternionBasics quaternionToTransform)
         Quaternion original = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion expected = new Quaternion();
         Quaternion actual = new Quaternion();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertQuaternionEquals(expected, actual, EPS);
      }

      { // Test transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
         Quaternion original = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion expected = new Quaternion();
         Quaternion actual = new Quaternion();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertQuaternionEquals(expected, actual, EPS);
      }

      { // Test transform(Vector4DBasics vectorToTransform)
         Vector4D original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D expected = new Vector4D();
         Vector4D actual = new Vector4D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         Vector4D original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D expected = new Vector4D();
         Vector4D actual = new Vector4D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      { // Test transform(Matrix3D matrixToTransform)
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D expected = new Matrix3D();
         Matrix3D actual = new Matrix3D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      { // Test transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D expected = new Matrix3D();
         Matrix3D actual = new Matrix3D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      { // Test transform(RotationMatrix matrixToTransform)
         RotationMatrix original = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         actual.set(original);
         qTransform.transform(actual);
         expected.set(original);
         rTransform.transform(expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      { // Test transform(Point2DBasics pointToTransform)
         Point2D original = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         actual.set(original);
         qTransform2D.transform(actual);
         rTransform2D.transform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
         actual.set(original);
         qTransform2D.transform(actual, true);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
         Point2D original = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         qTransform2D.transform(original, actual);
         rTransform2D.transform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
         qTransform2D.transform(original, actual, true);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DBasics vectorToTransform)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         actual.set(original);
         qTransform2D.transform(actual);
         rTransform2D.transform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         qTransform2D.transform(original, actual);
         rTransform2D.transform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testInverseTransform() throws Exception
   {
      Random random = new Random(34L);
      QuaternionBasedTransform qTransform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      RigidBodyTransform rTransform = new RigidBodyTransform(qTransform);

      RigidBodyTransform rTransform2D = new RigidBodyTransform();
      rTransform2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      rTransform2D.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
      QuaternionBasedTransform qTransform2D = new QuaternionBasedTransform(rTransform2D);

      { // Test inverseTransform(PointBasics pointToTransform)
         Point3D original = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         actual.set(original);
         qTransform.inverseTransform(actual);
         rTransform.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         Point3D original = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         qTransform.inverseTransform(original, actual);
         rTransform.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(VectorBasics vectorToTransform)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         actual.set(original);
         qTransform.inverseTransform(actual);
         rTransform.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         qTransform.inverseTransform(original, actual);
         rTransform.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(Point2DBasics pointToTransform)
         Point2D original = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         actual.set(original);
         qTransform2D.inverseTransform(actual);
         rTransform2D.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
         Point2D original = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         qTransform2D.inverseTransform(original, actual);
         rTransform2D.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DBasics vectorToTransform)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         actual.set(original);
         qTransform2D.inverseTransform(actual);
         rTransform2D.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         qTransform2D.inverseTransform(original, actual);
         rTransform2D.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformWithOtherRigidBodyTransform() throws Exception
   {
      Random random = new Random(23423L);

      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      RigidBodyTransform original = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expected = new RigidBodyTransform();
      RigidBodyTransform actual = new RigidBodyTransform();

      expected.set(transform);
      expected.multiply(original);

      transform.transform(original, actual);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testTransformWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(23423L);

      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      RigidBodyTransform originalRigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expectedRigidBodyTransform = new RigidBodyTransform();

      QuaternionBasedTransform original = new QuaternionBasedTransform(originalRigidBodyTransform);
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      transform.transform(originalRigidBodyTransform, expectedRigidBodyTransform);
      expected.set(expectedRigidBodyTransform);
      transform.transform(original, actual);

      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testTransformWithAffineTransform() throws Exception
   {
      Random random = new Random(23423L);

      QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      RigidBodyTransform originalRigidBodyTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform expectedRigidBodyTransform = new RigidBodyTransform();

      Vector3D scale = EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0);

      AffineTransform original = new AffineTransform(originalRigidBodyTransform);
      original.setScale(scale);
      AffineTransform expected = new AffineTransform();
      AffineTransform actual = new AffineTransform();

      transform.transform(originalRigidBodyTransform, expectedRigidBodyTransform);
      expected.set(expectedRigidBodyTransform);
      expected.setScale(scale);
      transform.transform(original, actual);

      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      QuaternionBasedTransform t1 = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      QuaternionBasedTransform t2 = new QuaternionBasedTransform();

      assertFalse(t1.equals(t2));
      assertFalse(t1.equals(null));
      assertFalse(t1.equals(new double[4]));
      t2.set(t1);
      assertTrue(t1.equals(t2));
      assertTrue(t1.equals(t2));

      double smallestEpsilon = 1.0e-16;
      double[] coeffs = new double[7];

      for (int index = 0; index < 3; index++)
      {
         t2.set(t1);
         assertTrue(t1.equals(t2));
         t1.get(coeffs);
         coeffs[index] += smallestEpsilon;
         t2.set(coeffs);
         assertFalse(t1.equals(t2));

         t2.set(t1);
         assertTrue(t1.equals(t2));
         t1.get(coeffs);
         coeffs[index] -= smallestEpsilon;
         t2.set(coeffs);
         assertFalse(t1.equals(t2));
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(2354L);
      QuaternionBasedTransform m1 = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
      QuaternionBasedTransform m2 = new QuaternionBasedTransform();
      double epsilon = 1.0e-3;
      double[] coeffs = new double[9];

      assertFalse(m1.epsilonEquals(m2, epsilon));
      m2.set(m1);
      assertTrue(m1.epsilonEquals(m2, epsilon));

      for (int index = 0; index < 3; index++)
      {
         m2.set(m1);
         assertTrue(m1.epsilonEquals(m2, epsilon));
         m1.get(coeffs);
         coeffs[index] += 0.999 * epsilon;
         m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6]);
         assertTrue(m1.epsilonEquals(m2, epsilon));

         m2.set(m1);
         assertTrue(m1.epsilonEquals(m2, epsilon));
         m1.get(coeffs);
         coeffs[index] += 1.001 * epsilon;
         m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6]);
         assertFalse(m1.epsilonEquals(m2, epsilon));

         m2.set(m1);
         assertTrue(m1.epsilonEquals(m2, epsilon));
         m1.get(coeffs);
         coeffs[index] -= 0.999 * epsilon;
         m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6]);
         assertTrue(m1.epsilonEquals(m2, epsilon));

         m2.set(m1);
         assertTrue(m1.epsilonEquals(m2, epsilon));
         m1.get(coeffs);
         coeffs[index] -= 1.001 * epsilon;
         m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6]);
         assertFalse(m1.epsilonEquals(m2, epsilon));
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(19825L);
      QuaternionBasedTransform qbtA;
      QuaternionBasedTransform qbtB;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         qbtA = createRandomTransform(random);
         double angleDiff = 0.99 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         Quaternion quaternion = new Quaternion(aa);
         quaternion.preMultiply(qbtA.getQuaternion());

         qbtB = new QuaternionBasedTransform(quaternion, qbtA.getTranslationVector());

         assertTrue(qbtA.geometricallyEquals(qbtB, epsilon));
         assertTrue(qbtB.geometricallyEquals(qbtA, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         qbtA = createRandomTransform(random);
         double angleDiff = 1.01 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         Quaternion quaternion = new Quaternion(aa);
         quaternion.preMultiply(qbtA.getQuaternion());

         qbtB = new QuaternionBasedTransform(quaternion, qbtA.getTranslationVector());

         assertFalse(qbtA.geometricallyEquals(qbtB, epsilon));
         assertFalse(qbtB.geometricallyEquals(qbtA, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         qbtA = createRandomTransform(random);

         Vector3D translation = new Vector3D(qbtA.getTranslationVector());
         Vector3D perturb = new Vector3D(translation);

         perturb.setX(translation.getX() + 0.9 * epsilon);
         qbtB = new QuaternionBasedTransform(new Quaternion(qbtA.getQuaternion()), perturb);

         assertTrue(qbtA.geometricallyEquals(qbtB, epsilon));
         assertTrue(qbtB.geometricallyEquals(qbtA, epsilon));

         perturb.setX(translation.getX() + 1.1 * epsilon);
         qbtB = new QuaternionBasedTransform(new Quaternion(qbtA.getQuaternion()), perturb);

         assertFalse(qbtA.geometricallyEquals(qbtB, epsilon));
         assertFalse(qbtB.geometricallyEquals(qbtA, epsilon));

         perturb = new Vector3D(translation);
         perturb.setY(translation.getY() + 0.9 * epsilon);
         qbtB = new QuaternionBasedTransform(new Quaternion(qbtA.getQuaternion()), perturb);

         assertTrue(qbtA.geometricallyEquals(qbtB, epsilon));
         assertTrue(qbtB.geometricallyEquals(qbtA, epsilon));

         perturb.setY(translation.getY() + 1.1 * epsilon);
         qbtB = new QuaternionBasedTransform(new Quaternion(qbtA.getQuaternion()), perturb);

         assertFalse(qbtA.geometricallyEquals(qbtB, epsilon));
         assertFalse(qbtB.geometricallyEquals(qbtA, epsilon));

         perturb = new Vector3D(translation);
         perturb.setZ(translation.getZ() + 0.9 * epsilon);
         qbtB = new QuaternionBasedTransform(new Quaternion(qbtA.getQuaternion()), perturb);

         assertTrue(qbtA.geometricallyEquals(qbtB, epsilon));
         assertTrue(qbtB.geometricallyEquals(qbtA, epsilon));

         perturb.setZ(translation.getZ() + 1.1 * epsilon);
         qbtB = new QuaternionBasedTransform(new Quaternion(qbtA.getQuaternion()), perturb);

         assertFalse(qbtA.geometricallyEquals(qbtB, epsilon));
         assertFalse(qbtB.geometricallyEquals(qbtA, epsilon));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         qbtA = createRandomTransform(random);
         double angleDiff = 1.01 * epsilon;
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleDiff);

         Quaternion quaternion = new Quaternion(aa);
         quaternion.preMultiply(qbtA.getQuaternion());

         qbtB = new QuaternionBasedTransform(quaternion, qbtA.getTranslationVector());

         assertFalse(qbtA.geometricallyEquals(qbtB, epsilon));
         assertFalse(qbtB.geometricallyEquals(qbtA, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(12345L);

      Quaternion quaternion;
      Vector3D translation;
      QuaternionBasedTransform qbt = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

      int previousHashCode, newHashCode;
      newHashCode = qbt.hashCode();
      assertEquals(newHashCode, qbt.hashCode());

      previousHashCode = qbt.hashCode();

      for (int i = 0; i < ITERATIONS; ++i)
      {
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         translation = EuclidCoreRandomTools.nextVector3D(random);
         qbt = new QuaternionBasedTransform(quaternion, translation);
         newHashCode = qbt.hashCode();
         assertNotEquals(previousHashCode, newHashCode);

         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testToString() throws Exception
   {
      Random random = new Random(12345L);

      QuaternionBasedTransform quaternionA;
      QuaternionBasedTransform quaternionB;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         quaternionA = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         quaternionB = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

         assertNotEquals(quaternionA.toString(), quaternionB.toString());
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         quaternionA = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         quaternionB = new QuaternionBasedTransform(quaternionA);

         assertEquals(quaternionA.toString(), quaternionB.toString());
      }
   }

   @Override
   public QuaternionBasedTransform createRandomTransform(Random random)
   {
      return EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
   }

   @Override
   public QuaternionBasedTransform createRandomTransform2D(Random random)
   {
      RigidBodyTransform rTransform2D = new RigidBodyTransform();
      rTransform2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      rTransform2D.setTranslation(EuclidCoreRandomTools.nextVector3D(random));
      QuaternionBasedTransform qTransform2D = new QuaternionBasedTransform(rTransform2D);
      return qTransform2D;
   }
}
