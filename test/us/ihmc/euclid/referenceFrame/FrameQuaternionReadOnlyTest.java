package us.ihmc.euclid.referenceFrame;

import org.junit.Test;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Tuple4DReadOnlyTest;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

import java.util.Random;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public abstract class FrameQuaternionReadOnlyTest<T extends FrameQuaternion> extends Tuple4DReadOnlyTest<T>
{
   private Random random = new Random(System.currentTimeMillis());

   private ReferenceFrame referenceFrame;
   private ReferenceFrame otherFrame;

   private FrameQuaternionReadOnly quaternion;

   private FrameQuaternionReadOnly fqro;
   private QuaternionReadOnly qro;
   private FrameQuaternion fq;
   private QuaternionBasics qb;
   private FrameTuple2DReadOnly ft2dro;
   private Tuple2DBasics t2db;
   private Tuple2DReadOnly t2dro;
   private FrameTuple2D ft2d;
   private FrameTuple3D ft3d0;
   private FrameTuple3D ft3d1;
   private FrameTuple3DReadOnly ft3dro;
   private Tuple3DReadOnly t3dro;
   private Tuple3DBasics t3db;
   private FrameVector3D fv3d;

   @Override
   public final T createEmptyTuple()
   {
      return createTuple(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0, 1.0);
   }

   public final T createEmptyTuple(ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, 0.0, 0.0, 0.0, 1.0);
   }

   @Override
   public final T createRandomTuple(Random random)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   @Override
   public final T createTuple(double x, double y, double z, double s)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), x, y, z, s);
   }

   public abstract T createTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s);

   @Override
   public double getEpsilon()
   {
      return 1e-10;
   }

   @Test(timeout = 30000)
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameQuaternionReadOnly.class, QuaternionReadOnly.class, true);
   }

   @Test(timeout = 30000)
   public void testReferenceFrameChecks()
   {
      // transform
      for (int i = 0; i < 100; ++i)
      {
         referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         quaternion = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);

         if (random.nextDouble() > 0.5)
         {
            ft2dro = EuclidFrameRandomTools.generateRandomFramePoint2D(random, referenceFrame);
            t2db = EuclidCoreRandomTools.generateRandomPoint2D(random);
            t2dro = EuclidCoreRandomTools.generateRandomPoint2D(random);
            ft2d = EuclidFrameRandomTools.generateRandomFramePoint2D(random, referenceFrame);
            ft3d0 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, referenceFrame);
            ft3d1 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, referenceFrame);
            ft3dro = EuclidFrameRandomTools.generateRandomFramePoint3D(random, referenceFrame);
            t3db = EuclidCoreRandomTools.generateRandomPoint3D(random);
            fqro = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);
            qb = EuclidCoreRandomTools.generateRandomQuaternion(random);
            fq = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);

            try
            {
               quaternion.transform(ft2dro, t2db, false);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.transform(t2dro, ft2d, false);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.transform(ft2dro, ft2d, false);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.transform(ft3d0);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.transform(ft3d0, ft3d1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.transform(ft3dro, ft3d1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.transform(ft3dro, t3db);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.transform(fqro, qb);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.transform(fqro, fq);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.transform(fq);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            quaternion = createEmptyTuple(referenceFrame);

            try
            {
               quaternion.transform(ft2d);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }
         }
         else
         {
            otherFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);

            ft2dro = EuclidFrameRandomTools.generateRandomFramePoint2D(random, otherFrame);
            t2db = EuclidCoreRandomTools.generateRandomPoint2D(random);
            ft2d = new FramePoint2D(otherFrame);
            ft3d0 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, otherFrame);
            ft3d1 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, otherFrame);
            ft3dro = EuclidFrameRandomTools.generateRandomFramePoint3D(random, otherFrame);
            t3db = EuclidCoreRandomTools.generateRandomPoint3D(random);
            fqro = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, otherFrame);
            qb = EuclidCoreRandomTools.generateRandomQuaternion(random);
            fq = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, otherFrame);

            try
            {
               quaternion.transform(ft2dro, t2db, false);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.transform(ft2dro, ft2d, false);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.transform(ft3d0);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.transform(ft3d0, ft3d1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.transform(ft3dro, ft3d1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.transform(ft3dro, t3db);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.transform(fqro, qb);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.transform(fqro, fq);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.transform(fq);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            quaternion = createEmptyTuple(referenceFrame);

            try
            {
               quaternion.transform(ft2d);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }
         }
      }

      // inverseTransform
      for (int i = 0; i < 100; ++i)
      {
         referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         quaternion = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);

         if (random.nextDouble() > 0.5)
         {
            ft2dro = EuclidFrameRandomTools.generateRandomFramePoint2D(random, referenceFrame);
            t2db = EuclidCoreRandomTools.generateRandomPoint2D(random);
            t2dro = EuclidCoreRandomTools.generateRandomPoint2D(random);
            ft2d = EuclidFrameRandomTools.generateRandomFramePoint2D(random, referenceFrame);
            ft3d0 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, referenceFrame);
            ft3d1 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, referenceFrame);
            ft3dro = EuclidFrameRandomTools.generateRandomFramePoint3D(random, referenceFrame);
            t3db = EuclidCoreRandomTools.generateRandomPoint3D(random);
            fqro = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);
            qb = EuclidCoreRandomTools.generateRandomQuaternion(random);
            fq = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);
            fv3d = EuclidFrameRandomTools.generateRandomFrameVector3D(random, referenceFrame);

            try
            {
               quaternion.inverseTransform(ft2dro, t2db, false);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.inverseTransform(t2dro, ft2d, false);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.inverseTransform(ft2dro, ft2d, false);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.inverseTransform(ft2d, false);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.inverseTransform(ft3d0);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.inverseTransform(ft3d0, ft3d1);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.inverseTransform(ft3dro, t3db);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.inverseTransform(fqro, qb);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.inverseTransform(fqro, fq);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try
            {
               quaternion.inverseTransform(fq);
            }
            catch (ReferenceFrameMismatchException excepted)
            {
               fail();
            }

            try {
               quaternion.inverseTransform(fv3d);
            } catch (ReferenceFrameMismatchException excepted) {
               fail();
            }

            quaternion = createEmptyTuple(referenceFrame);

            try {
               quaternion.inverseTransform(ft2dro, t2db);
            } catch (ReferenceFrameMismatchException excepted) {
               fail();
            }

            try {
               quaternion.inverseTransform(ft2dro, ft2d);
            } catch (ReferenceFrameMismatchException excepted) {
               fail();
            }

            try {
               quaternion.inverseTransform(t2dro, ft2d);
            } catch (ReferenceFrameMismatchException excepted) {
               fail();
            }

            try {
               quaternion.inverseTransform(ft2d);
            } catch (ReferenceFrameMismatchException excepted) {
               fail();
            }
         }
         else
         {
            otherFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);

            ft2dro = EuclidFrameRandomTools.generateRandomFramePoint2D(random, otherFrame);
            t2db = EuclidCoreRandomTools.generateRandomPoint2D(random);
            t2dro = EuclidCoreRandomTools.generateRandomPoint2D(random);
            ft2d = EuclidFrameRandomTools.generateRandomFramePoint2D(random, otherFrame);
            ft3d0 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, otherFrame);
            ft3d1 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, otherFrame);
            ft3dro = EuclidFrameRandomTools.generateRandomFramePoint3D(random, otherFrame);
            t3db = EuclidCoreRandomTools.generateRandomPoint3D(random);
            fqro = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, otherFrame);
            qb = EuclidCoreRandomTools.generateRandomQuaternion(random);
            fq = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, otherFrame);
            fv3d = EuclidFrameRandomTools.generateRandomFrameVector3D(random, otherFrame);

            try
            {
               quaternion.inverseTransform(ft2dro, t2db, false);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.inverseTransform(ft2dro, ft2d, false);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.inverseTransform(ft2d, false);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.inverseTransform(ft3d0);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.inverseTransform(ft3d0, ft3d1);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.inverseTransform(ft3dro, t3db);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.inverseTransform(fqro, qb);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.inverseTransform(fqro, fq);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try
            {
               quaternion.inverseTransform(fq);
               fail();
            }
            catch (ReferenceFrameMismatchException ignored)
            {

            }

            try {
               quaternion.inverseTransform(fv3d);
               fail();
            } catch (ReferenceFrameMismatchException ignored) {

            }

            quaternion = createEmptyTuple(referenceFrame);

            try {
               quaternion.inverseTransform(ft2dro, t2db);
               fail();
            } catch (ReferenceFrameMismatchException ignored) {

            }

            try {
               quaternion.inverseTransform(ft2dro, ft2d);
               fail();
            } catch (ReferenceFrameMismatchException ignored) {

            }
         }
      }

      // get
      for (int i = 0; i < 100; ++i) {
         referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         quaternion = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);

         if (random.nextDouble() > 0.5) {
            fv3d = EuclidFrameRandomTools.generateRandomFrameVector3D(random, referenceFrame);

            try {
               quaternion.get(fv3d);
            } catch (ReferenceFrameMismatchException excepted) {
               fail();
            }
         } else {
            fv3d = EuclidFrameRandomTools.generateRandomFrameVector3D(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try {
               quaternion.get(fv3d);
               fail();
            } catch (ReferenceFrameMismatchException ignored) {

            }
         }
      }

      // getEuler
      for (int i = 0; i < 100; ++i) {
         referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         quaternion = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);

         if (random.nextDouble() > 0.5) {
            fv3d = EuclidFrameRandomTools.generateRandomFrameVector3D(random, referenceFrame);

            try {
               quaternion.getEuler(fv3d);
            } catch (ReferenceFrameMismatchException excepted) {
               fail();
            }
         } else {
            fv3d = EuclidFrameRandomTools.generateRandomFrameVector3D(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try {
               quaternion.getEuler(fv3d);
               fail();
            } catch (ReferenceFrameMismatchException ignored) {

            }
         }
      }

      // distance
      for (int i = 0; i < 100; ++i) {
         referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         quaternion = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);

         if (random.nextDouble() > 0.5) {
            fqro = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);

            try {
               quaternion.distance(fqro);
            } catch (ReferenceFrameMismatchException excepted) {
               fail();
            }
         } else {
            fqro = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, EuclidFrameRandomTools.generateRandomReferenceFrame(random));

            try {
               quaternion.distance(fqro);
               fail();
            } catch (ReferenceFrameMismatchException ignored) {

            }
         }
      }
   }

   @Test(timeout = 30000)
   public void testReferenceFrameChanges() {
      referenceFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
      otherFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
      quaternion = createEmptyTuple(referenceFrame);

      for (int i = 0; i < 100; ++i)
      {
         t2dro = EuclidCoreRandomTools.generateRandomPoint2D(random);
         ft2d = EuclidFrameRandomTools.generateRandomFramePoint2D(random, otherFrame);

         quaternion.transform(t2dro, ft2d);

         assertTrue(quaternion.getReferenceFrame().equals(ft2d.getReferenceFrame()));

         ft2dro = EuclidFrameRandomTools.generateRandomFramePoint2D(random, referenceFrame);
         ft2d = EuclidFrameRandomTools.generateRandomFramePoint2D(random, otherFrame);

         quaternion.transform(ft2dro, ft2d);

         assertTrue(quaternion.getReferenceFrame().equals(ft2d.getReferenceFrame()));

         t3dro = EuclidCoreRandomTools.generateRandomPoint3D(random);
         ft3d0 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, otherFrame);

         quaternion.transform(t3dro, ft3d0);

         assertTrue(quaternion.getReferenceFrame().equals(ft3d0.getReferenceFrame()));

         ft3dro = EuclidFrameRandomTools.generateRandomFramePoint3D(random, referenceFrame);
         ft3d0 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, otherFrame);

         quaternion.transform(ft3dro, ft3d0);

         assertTrue(quaternion.getReferenceFrame().equals(ft3d0.getReferenceFrame()));

         qro = EuclidCoreRandomTools.generateRandomQuaternion(random);
         fq = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, otherFrame);

         quaternion.transform(qro, fq);

         assertTrue(quaternion.getReferenceFrame().equals(fq.getReferenceFrame()));

         fqro = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);
         fq = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, otherFrame);

         quaternion.transform(fqro, fq);

         assertTrue(quaternion.getReferenceFrame().equals(fq.getReferenceFrame()));

         t3dro = EuclidCoreRandomTools.generateRandomPoint3D(random);
         ft3d0 = EuclidFrameRandomTools.generateRandomFramePoint3D(random, otherFrame);

         quaternion.inverseTransform(t3dro, ft3d0);

         assertTrue(quaternion.getReferenceFrame().equals(ft3d0.getReferenceFrame()));

         t2dro = EuclidCoreRandomTools.generateRandomPoint2D(random);
         ft2d = EuclidFrameRandomTools.generateRandomFramePoint2D(random, otherFrame);

         quaternion.inverseTransform(t2dro, ft2d);

         assertTrue(quaternion.getReferenceFrame().equals(ft2d.getReferenceFrame()));

         ft2dro = EuclidFrameRandomTools.generateRandomFramePoint2D(random, referenceFrame);
         ft2d = EuclidFrameRandomTools.generateRandomFramePoint2D(random, otherFrame);

         quaternion.inverseTransform(ft2dro, ft2d);

         assertTrue(quaternion.getReferenceFrame().equals(ft2d.getReferenceFrame()));

         qro = EuclidCoreRandomTools.generateRandomQuaternion(random);
         fq = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, otherFrame);

         quaternion.inverseTransform(qro, fq);

         assertTrue(quaternion.getReferenceFrame().equals(fq.getReferenceFrame()));

         fqro = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, referenceFrame);
         fq = EuclidFrameRandomTools.generateRandomFrameQuaternion(random, otherFrame);

         quaternion.inverseTransform(fqro, fq);

         assertTrue(quaternion.getReferenceFrame().equals(fq.getReferenceFrame()));
      }
   }
}
