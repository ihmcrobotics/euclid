package us.ihmc.euclid.tools;

import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class TransformationToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testTransformationsWithMatrix3DReadOnly() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D matrix3D = EuclidCoreRandomTools.nextMatrix3D(random, 10.0);
         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Tuple3DBasics tupleExpected = new Point3D();
         matrix3D.transform(tupleOriginal, tupleExpected);

         Tuple3DBasics tupleActual = new Point3D();
         tupleActual.setX(TransformationTools.computeTransformedX(matrix3D, false, tupleOriginal));
         tupleActual.setY(TransformationTools.computeTransformedY(matrix3D, false, tupleOriginal));
         tupleActual.setZ(TransformationTools.computeTransformedZ(matrix3D, false, tupleOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

         Matrix3D transposedMatrix3D = new Matrix3D(matrix3D);
         transposedMatrix3D.transpose();
         transposedMatrix3D.transform(tupleOriginal, tupleExpected);
         tupleActual.setX(TransformationTools.computeTransformedX(matrix3D, true, tupleOriginal));
         tupleActual.setY(TransformationTools.computeTransformedY(matrix3D, true, tupleOriginal));
         tupleActual.setZ(TransformationTools.computeTransformedZ(matrix3D, true, tupleOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      }
   }

   @Test
   public void testTransformationsWithQuaternionReadOnly() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionReadOnly quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Tuple3DReadOnly tupleOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Tuple3DBasics tupleExpected = new Point3D();
         quaternion.transform(tupleOriginal, tupleExpected);

         Tuple3DBasics tupleActual = new Point3D();
         tupleActual.setX(TransformationTools.computeTransformedX(quaternion, false, tupleOriginal));
         tupleActual.setY(TransformationTools.computeTransformedY(quaternion, false, tupleOriginal));
         tupleActual.setZ(TransformationTools.computeTransformedZ(quaternion, false, tupleOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);

         Quaternion conjugatedQuaternion = new Quaternion(quaternion);
         conjugatedQuaternion.conjugate();
         conjugatedQuaternion.transform(tupleOriginal, tupleExpected);
         tupleActual.setX(TransformationTools.computeTransformedX(quaternion, true, tupleOriginal));
         tupleActual.setY(TransformationTools.computeTransformedY(quaternion, true, tupleOriginal));
         tupleActual.setZ(TransformationTools.computeTransformedZ(quaternion, true, tupleOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(tupleExpected, tupleActual, EPSILON);
      }
   }

   @Test
   public void testTransformationsWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Point3DReadOnly
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Point3DReadOnly pointOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3DBasics pointExpected = new Point3D();
         transform.transform(pointOriginal, pointExpected);

         Point3DBasics pointActual = new Point3D();
         pointActual.setX(TransformationTools.computeTransformedX(transform, false, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, false, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, false, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);

         RigidBodyTransform invertedTransform = new RigidBodyTransform(transform);
         invertedTransform.invert();
         invertedTransform.transform(pointOriginal, pointExpected);
         pointActual.setX(TransformationTools.computeTransformedX(transform, true, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, true, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, true, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Vector3DReadOnly
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Vector3DReadOnly vectorOriginal = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3DBasics vectorExpected = new Vector3D();
         transform.transform(vectorOriginal, vectorExpected);

         Vector3DBasics vectorActual = new Vector3D();
         vectorActual.setX(TransformationTools.computeTransformedX(transform, false, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, false, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, false, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);

         RigidBodyTransform invertedTransform = new RigidBodyTransform(transform);
         invertedTransform.invert();
         invertedTransform.transform(vectorOriginal, vectorExpected);
         vectorActual.setX(TransformationTools.computeTransformedX(transform, true, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, true, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, true, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);
      }
   }

   @Test
   public void testTransformationsWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Point3DReadOnly
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         Point3DReadOnly pointOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3DBasics pointExpected = new Point3D();
         transform.transform(pointOriginal, pointExpected);

         Point3DBasics pointActual = new Point3D();
         pointActual.setX(TransformationTools.computeTransformedX(transform, false, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, false, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, false, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);

         QuaternionBasedTransform invertedTransform = new QuaternionBasedTransform(transform);
         invertedTransform.invert();
         invertedTransform.transform(pointOriginal, pointExpected);
         pointActual.setX(TransformationTools.computeTransformedX(transform, true, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, true, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, true, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Vector3DReadOnly
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         Vector3DReadOnly vectorOriginal = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3DBasics vectorExpected = new Vector3D();
         transform.transform(vectorOriginal, vectorExpected);

         Vector3DBasics vectorActual = new Vector3D();
         vectorActual.setX(TransformationTools.computeTransformedX(transform, false, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, false, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, false, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);

         QuaternionBasedTransform invertedTransform = new QuaternionBasedTransform(transform);
         invertedTransform.invert();
         invertedTransform.transform(vectorOriginal, vectorExpected);
         vectorActual.setX(TransformationTools.computeTransformedX(transform, true, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, true, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, true, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);
      }
   }

   @Test
   public void testTransformationsWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Point3DReadOnly
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Point3DReadOnly pointOriginal = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3DBasics pointExpected = new Point3D();
         transform.transform(pointOriginal, pointExpected);

         Point3DBasics pointActual = new Point3D();
         pointActual.setX(TransformationTools.computeTransformedX(transform, false, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, false, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, false, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);

         transform.inverseTransform(pointOriginal, pointExpected);
         pointActual.setX(TransformationTools.computeTransformedX(transform, true, pointOriginal));
         pointActual.setY(TransformationTools.computeTransformedY(transform, true, pointOriginal));
         pointActual.setZ(TransformationTools.computeTransformedZ(transform, true, pointOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(pointExpected, pointActual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Vector3DReadOnly
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         Vector3DReadOnly vectorOriginal = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3DBasics vectorExpected = new Vector3D();
         transform.transform(vectorOriginal, vectorExpected);

         Vector3DBasics vectorActual = new Vector3D();
         vectorActual.setX(TransformationTools.computeTransformedX(transform, false, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, false, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, false, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);

         transform.inverseTransform(vectorOriginal, vectorExpected);
         vectorActual.setX(TransformationTools.computeTransformedX(transform, true, vectorOriginal));
         vectorActual.setY(TransformationTools.computeTransformedY(transform, true, vectorOriginal));
         vectorActual.setZ(TransformationTools.computeTransformedZ(transform, true, vectorOriginal));
         EuclidCoreTestTools.assertTuple3DEquals(vectorExpected, vectorActual, EPSILON);
      }
   }
}
