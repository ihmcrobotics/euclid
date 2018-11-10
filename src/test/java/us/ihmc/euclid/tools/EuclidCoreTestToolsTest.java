package us.ihmc.euclid.tools;

import static org.junit.Assert.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public class EuclidCoreTestToolsTest
{
   private static final String MESSAGE_PREFIX = "blop";
   private static final String FORMAT = EuclidCoreIOTools.getStringFormat(15, 12);
   private static final double EPSILON = 0.0001;

   @Test
   public void testAssertYawPitchRollEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertYawPitchRollEquals";
      Class<double[]> argumentsClass = double[].class;

      {
         double[] expected = null;
         double[] actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         double[] expected = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         double[] actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         double[] expected = null;
         double[] actual = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         double[] expected = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         double[] actual = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         double[] expected = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         double[] actual = expected.clone();
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertYawPitchRollGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertYawPitchRollGeometricallyEquals";
      Class<double[]> argumentsClass = double[].class;

      {
         double[] expected = null;
         double[] actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         double[] expected = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         double[] actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         double[] expected = null;
         double[] actual = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         double[] expected = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         double[] actual = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         double[] expected = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         double[] actual = expected.clone();
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertRotationVectorGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertRotationVectorGeometricallyEquals";
      Class<Vector3DReadOnly> argumentsClass = Vector3DReadOnly.class;

      {
         Vector3D expected = null;
         Vector3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = null;
         Vector3D actual = EuclidCoreRandomTools.nextVector3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = EuclidCoreRandomTools.nextVector3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = new Vector3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertTuple2DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertTuple2DEquals";
      Class<Tuple2DReadOnly> argumentsClass = Tuple2DReadOnly.class;

      {
         Vector2D expected = null;
         Vector2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector2D expected = null;
         Vector2D actual = EuclidCoreRandomTools.nextVector2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = EuclidCoreRandomTools.nextVector2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertPoint2DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertPoint2DGeometricallyEquals";
      Class<Point2DReadOnly> argumentsClass = Point2DReadOnly.class;

      {
         Point2D expected = null;
         Point2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Point2D expected = null;
         Point2D actual = EuclidCoreRandomTools.nextPoint2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D actual = EuclidCoreRandomTools.nextPoint2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D actual = new Point2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertVector2DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertVector2DGeometricallyEquals";
      Class<Vector2DReadOnly> argumentsClass = Vector2DReadOnly.class;

      {
         Vector2D expected = null;
         Vector2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector2D expected = null;
         Vector2D actual = EuclidCoreRandomTools.nextVector2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = EuclidCoreRandomTools.nextVector2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertTuple3DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertTuple3DEquals";
      Class<Tuple3DReadOnly> argumentsClass = Tuple3DReadOnly.class;

      {
         Vector3D expected = null;
         Vector3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = null;
         Vector3D actual = EuclidCoreRandomTools.nextVector3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = EuclidCoreRandomTools.nextVector3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = new Vector3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertPoint3DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertPoint3DGeometricallyEquals";
      Class<Point3DReadOnly> argumentsClass = Point3DReadOnly.class;

      {
         Point3D expected = null;
         Point3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Point3D expected = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Point3D expected = null;
         Point3D actual = EuclidCoreRandomTools.nextPoint3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Point3D expected = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D actual = EuclidCoreRandomTools.nextPoint3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Point3D expected = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D actual = new Point3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertVector3DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertVector3DGeometricallyEquals";
      Class<Vector3DReadOnly> argumentsClass = Vector3DReadOnly.class;

      {
         Vector3D expected = null;
         Vector3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = null;
         Vector3D actual = EuclidCoreRandomTools.nextVector3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = EuclidCoreRandomTools.nextVector3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = new Vector3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertTuple4DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertTuple4DEquals";
      Class<Tuple4DReadOnly> argumentsClass = Tuple4DReadOnly.class;

      {
         Vector4D expected = null;
         Vector4D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector4D expected = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector4D expected = null;
         Vector4D actual = EuclidCoreRandomTools.nextVector4D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector4D expected = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = EuclidCoreRandomTools.nextVector4D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector4D expected = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = new Vector4D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertVector4DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertVector4DGeometricallyEquals";
      Class<Vector4DReadOnly> argumentsClass = Vector4DReadOnly.class;

      {
         Vector4D expected = null;
         Vector4D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector4D expected = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector4D expected = null;
         Vector4D actual = EuclidCoreRandomTools.nextVector4D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector4D expected = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = EuclidCoreRandomTools.nextVector4D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Vector4D expected = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = new Vector4D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertMatrix3DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertMatrix3DEquals";
      Class<Matrix3DReadOnly> argumentsClass = Matrix3DReadOnly.class;

      {
         Matrix3D expected = null;
         Matrix3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Matrix3D expected = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Matrix3D expected = null;
         Matrix3D actual = EuclidCoreRandomTools.nextMatrix3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Matrix3D expected = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = EuclidCoreRandomTools.nextMatrix3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Matrix3D expected = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = new Matrix3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertRotationMatrixGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertRotationMatrixGeometricallyEquals";
      Class<RotationMatrixReadOnly> argumentsClass = RotationMatrixReadOnly.class;

      {
         RotationMatrix expected = null;
         RotationMatrix actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RotationMatrix expected = null;
         RotationMatrix actual = EuclidCoreRandomTools.nextRotationMatrix(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = EuclidCoreRandomTools.nextRotationMatrix(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertQuaternionEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertQuaternionEquals";
      Class<QuaternionReadOnly> argumentsClass = QuaternionReadOnly.class;

      {
         Quaternion expected = null;
         Quaternion actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Quaternion expected = null;
         Quaternion actual = EuclidCoreRandomTools.nextQuaternion(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion actual = EuclidCoreRandomTools.nextQuaternion(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion actual = new Quaternion(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertQuaternionGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertQuaternionGeometricallyEquals";
      Class<QuaternionReadOnly> argumentsClass = QuaternionReadOnly.class;

      {
         Quaternion expected = null;
         Quaternion actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Quaternion expected = null;
         Quaternion actual = EuclidCoreRandomTools.nextQuaternion(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion actual = EuclidCoreRandomTools.nextQuaternion(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion actual = new Quaternion(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertAxisAngleEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertAxisAngleEquals";
      Class<AxisAngleReadOnly> argumentsClass = AxisAngleReadOnly.class;

      {
         AxisAngle expected = null;
         AxisAngle actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         AxisAngle actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AxisAngle expected = null;
         AxisAngle actual = EuclidCoreRandomTools.nextAxisAngle(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         AxisAngle actual = EuclidCoreRandomTools.nextAxisAngle(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         AxisAngle actual = new AxisAngle(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertAxisAngleGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertAxisAngleGeometricallyEquals";
      Class<AxisAngleReadOnly> argumentsClass = AxisAngleReadOnly.class;

      {
         AxisAngle expected = null;
         AxisAngle actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         AxisAngle actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AxisAngle expected = null;
         AxisAngle actual = EuclidCoreRandomTools.nextAxisAngle(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         AxisAngle actual = EuclidCoreRandomTools.nextAxisAngle(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         AxisAngle actual = new AxisAngle(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertRigidBodyTransformEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertRigidBodyTransformEquals";
      Class<RigidBodyTransform> argumentsClass = RigidBodyTransform.class;

      {
         RigidBodyTransform expected = null;
         RigidBodyTransform actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RigidBodyTransform expected = null;
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertRigidBodyTransformGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertRigidBodyTransformGeometricallyEquals";
      Class<RigidBodyTransform> argumentsClass = RigidBodyTransform.class;

      {
         RigidBodyTransform expected = null;
         RigidBodyTransform actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RigidBodyTransform expected = null;
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertQuaternionBasedTransformEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertQuaternionBasedTransformEquals";
      Class<QuaternionBasedTransform> argumentsClass = QuaternionBasedTransform.class;

      {
         QuaternionBasedTransform expected = null;
         QuaternionBasedTransform actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         QuaternionBasedTransform expected = null;
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = new QuaternionBasedTransform(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertQuaternionBasedTransformGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertQuaternionBasedTransformGeometricallyEquals";
      Class<QuaternionBasedTransform> argumentsClass = QuaternionBasedTransform.class;

      {
         QuaternionBasedTransform expected = null;
         QuaternionBasedTransform actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         QuaternionBasedTransform expected = null;
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         QuaternionBasedTransform expected = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = new QuaternionBasedTransform(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertAffineTransformEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertAffineTransformEquals";
      Class<AffineTransform> argumentsClass = AffineTransform.class;

      {
         AffineTransform expected = null;
         AffineTransform actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AffineTransform expected = null;
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = new AffineTransform(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertAffineTransformGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertAffineTransformGeometricallyEquals";
      Class<AffineTransform> argumentsClass = AffineTransform.class;

      {
         AffineTransform expected = null;
         AffineTransform actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AffineTransform expected = null;
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = EuclidCoreRandomTools.nextAffineTransform(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         AffineTransform expected = EuclidCoreRandomTools.nextAffineTransform(random);
         AffineTransform actual = new AffineTransform(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   private static void assertAssertionMethodsBehaveProperly(boolean failExpected, String methodName, Class<?> argumentsClass, Object expected, Object actual,
                                                            double epsilon)
         throws Throwable
   {
      assertAssertionMethodsBehaveProperly(EuclidCoreTestTools.class, failExpected, methodName, argumentsClass, expected, actual, epsilon);
   }

   public static void assertAssertionMethodsBehaveProperly(Class<?> assertionClassHolder, boolean failExpected, String methodName, Class<?> argumentsClass,
                                                           Object expected, Object actual, double epsilon)
         throws Throwable
   {
      Method assertionMethod1 = assertionClassHolder.getMethod(methodName, argumentsClass, argumentsClass, Double.TYPE);
      Method assertionMethod2 = assertionClassHolder.getMethod(methodName, String.class, argumentsClass, argumentsClass, Double.TYPE);
      Method assertionMethod3 = assertionClassHolder.getMethod(methodName, String.class, argumentsClass, argumentsClass, Double.TYPE, String.class);

      try
      {
         try
         {
            assertionMethod1.invoke(null, expected, actual, epsilon);
         }
         catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            handleInvokationExceptions(methodName, e);
         }

         if (failExpected)
            didNotThrowAssertionError();
      }
      catch (AssertionError e)
      {
         if (!failExpected)
            unexpectedlyThrewAssertionError(e);
         // good
      }

      try
      {
         try
         {
            assertionMethod2.invoke(null, MESSAGE_PREFIX, expected, actual, epsilon);
         }
         catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            handleInvokationExceptions(methodName, e);
         }

         if (failExpected)
            didNotThrowAssertionError();
      }
      catch (AssertionError e)
      {
         if (!failExpected)
            unexpectedlyThrewAssertionError(e);
         // good
         assertTrue(e.getMessage().startsWith(MESSAGE_PREFIX));
      }

      try
      {
         try
         {
            assertionMethod3.invoke(null, MESSAGE_PREFIX, expected, actual, epsilon, FORMAT);
         }
         catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            handleInvokationExceptions(methodName, e);
         }

         if (failExpected)
            didNotThrowAssertionError();
      }
      catch (AssertionError e)
      {
         if (!failExpected)
            unexpectedlyThrewAssertionError(e);
         // good
         assertTrue(e.getMessage().startsWith(MESSAGE_PREFIX));
      }
   }

   private static void handleInvokationExceptions(String methodName, Exception e) throws Throwable
   {
      if (e instanceof InvocationTargetException)
      {
         InvocationTargetException invocationTargetException = (InvocationTargetException) e;
         if (invocationTargetException.getTargetException() instanceof AssertionError)
         {
            throw (AssertionError) invocationTargetException.getTargetException();
         }
         else
         {
            throw invocationTargetException.getTargetException();
         }
      }
      throw e;
   }

   private static void unexpectedlyThrewAssertionError(AssertionError e)
   {
      fail("Should NOT have thrown an " + AssertionError.class.getSimpleName() + ": " + e);
   }

   private static void didNotThrowAssertionError()
   {
      fail("Should have thrown an " + AssertionError.class.getSimpleName());
   }
}
