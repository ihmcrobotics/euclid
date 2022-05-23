package us.ihmc.euclid.yawPitchRoll;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;
import static us.ihmc.euclid.tools.EuclidCoreTestTools.assertExceptionIsThrown;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.YawPitchRollTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

public abstract class YawPitchRollReadOnlyTest<T extends YawPitchRollReadOnly>
{
   public abstract T createEmptyYawPitchRoll();

   public abstract T createYawPitchRoll(Orientation3DReadOnly orientation3D);

   public abstract T createYawPitchRoll(double yaw, double pitch, double roll);

   public abstract T createRandomYawPitchRoll(Random random);

   public abstract double getEpsilon();

   public abstract double getSmallestEpsilon();

   @Test
   public void testComponentGetters()
   {
      Random random = new Random(346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double expectedYaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double expectedPitch = EuclidCoreRandomTools.nextDouble(random, 0.5 * Math.PI);
         double expectedRoll = EuclidCoreRandomTools.nextDouble(random, Math.PI);

         T ypr = createYawPitchRoll(expectedYaw, expectedPitch, expectedRoll);
         assertEquals(expectedYaw, ypr.getYaw(), getEpsilon());
         assertEquals(expectedPitch, ypr.getPitch(), getEpsilon());
         assertEquals(expectedRoll, ypr.getRoll(), getEpsilon());

         assertEquals((float) expectedYaw, ypr.getYaw32(), getEpsilon());
         assertEquals((float) expectedPitch, ypr.getPitch32(), getEpsilon());
         assertEquals((float) expectedRoll, ypr.getRoll32(), getEpsilon());
      }
   }

   @Test
   public void testContainsNaN()
   {
      T ypr;

      ypr = createYawPitchRoll(0.0, 0.0, 0.0);
      assertFalse(ypr.containsNaN());
      ypr = createYawPitchRoll(Double.NaN, 0.0, 0.0);
      assertTrue(ypr.containsNaN());
      ypr = createYawPitchRoll(0.0, Double.NaN, 0.0);
      assertTrue(ypr.containsNaN());
      ypr = createYawPitchRoll(0.0, 0.0, Double.NaN);
      assertTrue(ypr.containsNaN());
   }

   @Test
   public void testIsZero()
   {
      Random random = new Random(342);
      double yaw, pitch, roll;

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();
         yaw = pitch = roll = 0.0;
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
         yaw = pitch = roll = 0.0;
         yaw = Double.NaN;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
         yaw = pitch = roll = 0.0;
         pitch = Double.NaN;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
         yaw = pitch = roll = 0.0;
         roll = Double.NaN;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));

         yaw = pitch = roll = 0.0;
         yaw = epsilon;
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
         yaw = pitch = roll = 0.0;
         pitch = epsilon;
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
         yaw = pitch = roll = 0.0;
         roll = epsilon;
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));

         yaw = pitch = roll = 0.0;
         yaw = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
         yaw = pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
         yaw = pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));

         yaw = pitch = roll = 0.0;
         yaw = 1.01 * epsilon;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
         yaw = pitch = roll = 0.0;
         pitch = 1.01 * epsilon;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
         yaw = pitch = roll = 0.0;
         roll = 1.01 * epsilon;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isZero(epsilon));
      }
   }

   @Test
   public void testIsOrientation2D()
   {
      Random random = new Random(342);
      double yaw, pitch, roll;

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();
         yaw = pitch = roll = 0.0;
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));
         yaw = pitch = roll = 0.0;
         yaw = Double.NaN;
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));
         yaw = pitch = roll = 0.0;
         pitch = Double.NaN;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));
         yaw = pitch = roll = 0.0;
         roll = Double.NaN;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));

         yaw = EuclidCoreRandomTools.nextDouble(random, 100.0);
         pitch = roll = 0.0;
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));
         pitch = roll = 0.0;
         pitch = epsilon;
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));
         pitch = roll = 0.0;
         roll = epsilon;
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));

         pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));
         pitch = roll = 0.0;
         pitch = EuclidCoreRandomTools.nextDouble(random, epsilon);
         assertTrue(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));

         pitch = roll = 0.0;
         pitch = 1.01 * epsilon;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));
         pitch = roll = 0.0;
         roll = 1.01 * epsilon;
         assertFalse(createYawPitchRoll(yaw, pitch, roll).isOrientation2D(epsilon));
      }
   }

   @Test
   public void testDistance() throws Exception
   {
      Random random = new Random(345093);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T ypr1 = createRandomYawPitchRoll(random);
         T ypr2 = createRandomYawPitchRoll(random);

         double actual = ypr1.distance(ypr2);
         double expected = YawPitchRollTools.distance(ypr1, ypr2);
         assertEquals(expected, actual, getEpsilon());
      }
      
      for (int i = 0; i < ITERATIONS; ++i)
      {
         YawPitchRoll yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);
         Quaternion quaternion = new Quaternion(yawPitchRoll);
         Orientation3DBasics orientation = EuclidCoreRandomTools.nextOrientation3D(random);
         double actual = yawPitchRoll.distance(orientation);
         double expected = quaternion.distance(orientation);
         assertEquals(actual, expected, getEpsilon());
      }
   }
   
   @Test
   public void testAngle() throws Exception
   {
      Random random = new Random(32423);
      for (int i = 0; i < ITERATIONS; ++i)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         double angle = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         axisAngle.setAngle(angle);
         YawPitchRoll yawPitchRoll = new YawPitchRoll(axisAngle);
         assertEquals(Math.abs(angle), yawPitchRoll.angle(),getEpsilon());
      }
   }

   @Test
   public void testOrientationGetters() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(RotationMatrix rotationMatrixToPack)
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         T ypr = createRandomYawPitchRoll(random);

         ypr.get(actual);

         RotationMatrixConversion.convertYawPitchRollToMatrix(ypr, expected);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(AxisAngleBasics axisAngleToPack)
         AxisAngleBasics expected = new AxisAngle();
         AxisAngleBasics actual = new AxisAngle();

         T ypr = createRandomYawPitchRoll(random);

         ypr.get(actual);

         AxisAngleConversion.convertYawPitchRollToAxisAngle(ypr, expected);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(QuaternionBasics quaternionToPack)
         QuaternionBasics expected = new Quaternion();
         QuaternionBasics actual = new Quaternion();

         T ypr = createRandomYawPitchRoll(random);

         ypr.get(actual);

         QuaternionConversion.convertYawPitchRollToQuaternion(ypr, expected);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(YawPitchRollBasics yawPitchRollToPack)
         YawPitchRollBasics expected = new YawPitchRoll();
         YawPitchRollBasics actual = new YawPitchRoll();

         T ypr = createRandomYawPitchRoll(random);

         ypr.get(actual);

         expected.set(ypr.getYaw(), ypr.getPitch(), ypr.getRoll());
         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testGetRotationVector() throws Exception
   {
      Random random = new Random(435983);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test getRotationVector(Vector3DBasics rotationVectorToPack)
         Vector3DBasics expected = new Vector3D();
         Vector3DBasics actual = new Vector3D();

         T ypr = createRandomYawPitchRoll(random);

         ypr.getRotationVector(actual);

         RotationVectorConversion.convertYawPitchRollToRotationVector(ypr, expected);
         EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testGetEuler() throws Exception
   {
      Random random = new Random(435983);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);
         T ypr = createYawPitchRoll(q);

         Tuple3DBasics expected = new Point3D();
         Tuple3DBasics actual = new Point3D();

         q.getEuler(expected);
         ypr.getEuler(actual);

         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testGetDoubleArray()
   {
      Random random = new Random(3513515L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(double[] yawPitchRollArray)
         double[] yprArray = new double[4];
         T ypr = createRandomYawPitchRoll(random);
         ypr.get(yprArray);

         assertTrue(ypr.getYaw() == yprArray[0]);
         assertTrue(ypr.getPitch() == yprArray[1]);
         assertTrue(ypr.getRoll() == yprArray[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startIndex, double[] yawPitchRollArray)
         int startIndex = random.nextInt(20);
         double[] yprArray = new double[startIndex + 4 + random.nextInt(10)];
         T ypr = createRandomYawPitchRoll(random);

         ypr.get(startIndex, yprArray);

         assertTrue(ypr.getYaw() == yprArray[startIndex + 0]);
         assertTrue(ypr.getPitch() == yprArray[startIndex + 1]);
         assertTrue(ypr.getRoll() == yprArray[startIndex + 2]);
      }
   }

   @Test
   public void testGetFloatArray()
   {
      Random random = new Random(3513515L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(float[] yawPitchRollArray)
         float[] yprArray = new float[4];
         T ypr = createRandomYawPitchRoll(random);
         ypr.get(yprArray);

         assertTrue(ypr.getYaw32() == yprArray[0]);
         assertTrue(ypr.getPitch32() == yprArray[1]);
         assertTrue(ypr.getRoll32() == yprArray[2]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test get(int startIndex, float[] yawPitchRollArray)
         int startIndex = random.nextInt(20);
         float[] yprArray = new float[startIndex + 4 + random.nextInt(10)];
         T ypr = createRandomYawPitchRoll(random);

         ypr.get(startIndex, yprArray);

         assertTrue(ypr.getYaw32() == yprArray[startIndex + 0]);
         assertTrue(ypr.getPitch32() == yprArray[startIndex + 1]);
         assertTrue(ypr.getRoll32() == yprArray[startIndex + 2]);
      }
   }

   @Test
   public void testGetElement() throws Exception
   {
      Random random = new Random(324234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T ypr = createRandomYawPitchRoll(random);
         assertTrue(ypr.getYaw() == ypr.getElement(0));
         assertTrue(ypr.getPitch() == ypr.getElement(1));
         assertTrue(ypr.getRoll() == ypr.getElement(2));

         assertTrue(ypr.getYaw32() == ypr.getElement32(0));
         assertTrue(ypr.getPitch32() == ypr.getElement32(1));
         assertTrue(ypr.getRoll32() == ypr.getElement32(2));
      }
   }

   @Test
   public void testTransform()
   {
      Random random = new Random(24546654);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         T ypr = createRandomYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         ypr.transform(tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         T ypr = createRandomYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         YawPitchRollTools.addTransform(ypr, tupleOriginal, actual);
         new RotationMatrix(ypr).addTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
         T ypr = createRandomYawPitchRoll(random);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = new Point2D();
         Tuple2DBasics expected = new Point2D();

         ypr.transform(tupleOriginal, actual, false);
         new RotationMatrix(ypr).transform(tupleOriginal, expected, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, getEpsilon());

         assertExceptionIsThrown(() -> createRandomYawPitchRoll(random).transform(tupleOriginal, actual, true), NotAnOrientation2DException.class);
         assertExceptionIsThrown(() -> createYawPitchRoll(1.0, 0.0, 1.0).transform(tupleOriginal, actual, true), NotAnOrientation2DException.class);
         assertExceptionIsThrown(() -> createYawPitchRoll(1.0, 1.0, 0.0).transform(tupleOriginal, actual, true), NotAnOrientation2DException.class);
         ypr = createYawPitchRoll(random.nextDouble(), 0.0, 0.0);
         ypr.transform(tupleOriginal, actual, true);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Matrix3DReadOnly tupleOriginal, Matrix3DBasics tupleTransformed)
         T ypr = createRandomYawPitchRoll(random);

         Matrix3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DBasics actual = new Matrix3D();
         Matrix3DBasics expected = new Matrix3D();

         ypr.transform(tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(RotationMatrixReadOnly tupleOriginal, RotationMatrixBasics tupleTransformed)
         T ypr = createRandomYawPitchRoll(random);

         RotationMatrixReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix();
         RotationMatrix expected = new RotationMatrix();

         ypr.transform(tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test transform(Vector4DReadOnly tupleOriginal, Vector4DBasics tupleTransformed)
         T ypr = createRandomYawPitchRoll(random);

         Vector4DReadOnly tupleOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D();
         Vector4DBasics expected = new Vector4D();

         ypr.transform(tupleOriginal, actual);
         new RotationMatrix(ypr).transform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testInverseTransform()
   {
      Random random = new Random(24546654);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleinverseTransformed)
         T ypr = createRandomYawPitchRoll(random);

         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DBasics actual = new Point3D();
         Tuple3DBasics expected = new Point3D();

         ypr.inverseTransform(tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleinverseTransformed)
         T ypr = createRandomYawPitchRoll(random);

         Tuple2DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DBasics actual = new Point2D();
         Tuple2DBasics expected = new Point2D();

         ypr.inverseTransform(tupleOriginal, actual, false);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected, false);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, getEpsilon());

         assertExceptionIsThrown(() -> createRandomYawPitchRoll(random).inverseTransform(tupleOriginal, actual, true), NotAnOrientation2DException.class);
         assertExceptionIsThrown(() -> createYawPitchRoll(1.0, 0.0, 1.0).inverseTransform(tupleOriginal, actual, true), NotAnOrientation2DException.class);
         assertExceptionIsThrown(() -> createYawPitchRoll(1.0, 1.0, 0.0).inverseTransform(tupleOriginal, actual, true), NotAnOrientation2DException.class);
         ypr = createYawPitchRoll(random.nextDouble(), 0.0, 0.0);
         ypr.inverseTransform(tupleOriginal, actual, true);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Matrix3DReadOnly tupleOriginal, Matrix3DBasics tupleinverseTransformed)
         T ypr = createRandomYawPitchRoll(random);

         Matrix3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3DBasics actual = new Matrix3D();
         Matrix3DBasics expected = new Matrix3D();

         ypr.inverseTransform(tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(RotationMatrixReadOnly tupleOriginal, RotationMatrixBasics tupleinverseTransformed)
         T ypr = createRandomYawPitchRoll(random);

         RotationMatrixReadOnly tupleOriginal = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix();
         RotationMatrix expected = new RotationMatrix();

         ypr.inverseTransform(tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test inverseTransform(Vector4DReadOnly tupleOriginal, Vector4DBasics tupleinverseTransformed)
         T ypr = createRandomYawPitchRoll(random);

         Vector4DReadOnly tupleOriginal = EuclidCoreRandomTools.nextVector4D(random);
         Vector4DBasics actual = new Vector4D();
         Vector4DBasics expected = new Vector4D();

         ypr.inverseTransform(tupleOriginal, actual);
         new RotationMatrix(ypr).inverseTransform(tupleOriginal, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);

      T ypr = createRandomYawPitchRoll(random);

      assertFalse(ypr.equals(createEmptyYawPitchRoll()));
      Object emptyYawPitchRollAsObject = createEmptyYawPitchRoll();
      assertFalse(ypr.equals(emptyYawPitchRollAsObject));
      Object yprAsObject = ypr;
      assertTrue(ypr.equals(yprAsObject));
      assertFalse(ypr.equals(null));
      assertFalse(ypr.equals(new double[5]));

      double yaw = ypr.getYaw();
      double pitch = ypr.getPitch();
      double roll = ypr.getRoll();

      assertTrue(ypr.equals(createYawPitchRoll(yaw, pitch, roll)));

      assertFalse(ypr.equals(createYawPitchRoll(yaw + getSmallestEpsilon(), pitch, roll)));
      assertFalse(ypr.equals(createYawPitchRoll(yaw - getSmallestEpsilon(), pitch, roll)));

      assertFalse(ypr.equals(createYawPitchRoll(yaw, pitch + getSmallestEpsilon(), roll)));
      assertFalse(ypr.equals(createYawPitchRoll(yaw, pitch - getSmallestEpsilon(), roll)));

      assertFalse(ypr.equals(createYawPitchRoll(yaw, pitch, roll + getSmallestEpsilon())));
      assertFalse(ypr.equals(createYawPitchRoll(yaw, pitch, roll - getSmallestEpsilon())));
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      T ypr = createRandomYawPitchRoll(random);
      double yaw = ypr.getYaw();
      double pitch = ypr.getPitch();
      double roll = ypr.getRoll();

      assertTrue(ypr.epsilonEquals(createYawPitchRoll(yaw + 0.999 * epsilon, pitch, roll), epsilon));
      assertTrue(ypr.epsilonEquals(createYawPitchRoll(yaw - 0.999 * epsilon, pitch, roll), epsilon));

      assertTrue(ypr.epsilonEquals(createYawPitchRoll(yaw, pitch + 0.999 * epsilon, roll), epsilon));
      assertTrue(ypr.epsilonEquals(createYawPitchRoll(yaw, pitch - 0.999 * epsilon, roll), epsilon));

      assertTrue(ypr.epsilonEquals(createYawPitchRoll(yaw, pitch, roll + 0.999 * epsilon), epsilon));
      assertTrue(ypr.epsilonEquals(createYawPitchRoll(yaw, pitch, roll - 0.999 * epsilon), epsilon));

      assertFalse(ypr.epsilonEquals(createYawPitchRoll(yaw + 1.001 * epsilon, pitch, roll), epsilon));
      assertFalse(ypr.epsilonEquals(createYawPitchRoll(yaw - 1.001 * epsilon, pitch, roll), epsilon));

      assertFalse(ypr.epsilonEquals(createYawPitchRoll(yaw, pitch + 1.001 * epsilon, roll), epsilon));
      assertFalse(ypr.epsilonEquals(createYawPitchRoll(yaw, pitch - 1.001 * epsilon, roll), epsilon));

      assertFalse(ypr.epsilonEquals(createYawPitchRoll(yaw, pitch, roll + 1.001 * epsilon), epsilon));
      assertFalse(ypr.epsilonEquals(createYawPitchRoll(yaw, pitch, roll - 1.001 * epsilon), epsilon));
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(35454L);

      T yprbA;
      YawPitchRoll yprbB;
      
      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         yprbA = createRandomYawPitchRoll(random);
         double angleEps = epsilon * 1.01;
         T ypr = createYawPitchRoll(new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleEps));

         yprbB = new YawPitchRoll(ypr);
         yprbB.prepend(yprbA);
         
         System.out.println(yprbA.angle() * 180 / Math.PI);
         System.out.println(yprbB.angle() * 180 / Math.PI);

         assertFalse(yprbA.geometricallyEquals(yprbB, epsilon));
      }
      
      System.out.println("---------------------------------------------------");
      System.out.println("---------------------------------------------------");
      System.out.println("---------------------------------------------------");

      
      // Fail -->
      for (int i = 0; i < ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         yprbA = createRandomYawPitchRoll(random);
         double angleEps = epsilon * 0.99;
         T ypr = createYawPitchRoll(new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), angleEps));

         yprbB = new YawPitchRoll(ypr);
         yprbB.prepend(yprbA);
         
         System.out.println(yprbA.angle() * 180 / Math.PI);
         System.out.println(yprbB.angle() * 180 / Math.PI);
         
         assertTrue(yprbA.geometricallyEquals(yprbB, epsilon));
      }
   }
}
