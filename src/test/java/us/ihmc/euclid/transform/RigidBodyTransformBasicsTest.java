package us.ihmc.euclid.transform;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;

public abstract class RigidBodyTransformBasicsTest<T extends RigidBodyTransformBasics> extends TransformTest<T>
{
   private static final double EPS = 1.0e-14;

   @Override
   public abstract T createRandomTransform(Random random);

   @Override
   public abstract T createRandomTransform2D(Random random);

   public abstract T copy(T original);

   public abstract T identity();

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(465416L);

      // Test against invert
      for (int i = 0; i < ITERATIONS; i++)
      {
         T transform = createRandomTransform(random);
         T inverse = copy(transform);
         inverse.invert();

         assertTrue(transform.hasRotation());
         assertTrue(transform.hasTranslation());
         transform.multiply(inverse);
         assertFalse(transform.hasRotation());
         assertFalse(transform.hasTranslation());

         EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(transform, identity(), EPS);
      }

      // Test against EJML
      for (int i = 0; i < ITERATIONS; i++)
      {
         T t1 = createRandomTransform(random);
         T t2 = createRandomTransform(random);
         checkMultiplyAgainstEJML(t1, t2);
      }

      /*
      // Try different combinations with/without translation/rotation
      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         RigidBodyTransform t2 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(new Quaternion(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertFalse(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertFalse(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(new RotationMatrix(), EuclidCoreRandomTools.nextVector3D(random));
         checkMultiplyAgainstEJML(t1, t2);
         assertFalse(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertFalse(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), new Vector3D());
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertFalse(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertFalse(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2.getRotation().set(t1.getRotation());
         t2.invertRotation();
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertFalse(t1.hasRotation());
         assertTrue(t1.hasTranslation());

         t1 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         t2 = new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextVector3D(random));
         Vector3D negateTranslation = new Vector3D(t1.getTranslation());
         negateTranslation.negate();
         t1.inverseTransform(negateTranslation);
         t2.getTranslation().set(negateTranslation);
         checkMultiplyAgainstEJML(t1, t2);
         assertTrue(t1.hasRotation());
         assertTrue(t1.hasTranslation());
         t1.multiply(t2);
         assertTrue(t1.hasRotation());
         assertFalse(t1.hasTranslation());
      }
      */
   }

   private void checkMultiplyAgainstEJML(T t1, T t2)
   {
      T actual_t3 = copy(t1);
      actual_t3.multiply(t2);

      DMatrixRMaj m1 = CommonOps_DDRM.identity(4);
      DMatrixRMaj m2 = CommonOps_DDRM.identity(4);
      DMatrixRMaj m3 = new DMatrixRMaj(4, 4);

      // Pack t1 into m1
      Matrix3D r1 = new Matrix3D();
      t1.getRotation().get(r1);
      r1.get(m1);
      t1.getTranslation().get(0, 3, m1);

      // Pack t2 into m2
      Matrix3D r2 = new Matrix3D();
      t2.getRotation().get(r2);
      r2.get(m2);
      t2.getTranslation().get(0, 3, m2);

      CommonOps_DDRM.mult(m1, m2, m3);

      T expected_t3 = identity();
      expected_t3.getRotation().setRotationMatrix(m3.get(0, 0), m3.get(0, 1), m3.get(0, 2),
                                                  m3.get(1, 0), m3.get(1, 1), m3.get(1, 2),
                                                  m3.get(2, 0), m3.get(2, 1), m3.get(2, 2));
      expected_t3.getTranslation().set(0, 3, m3);

      EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(expected_t3, actual_t3, EPS);
   }

}
