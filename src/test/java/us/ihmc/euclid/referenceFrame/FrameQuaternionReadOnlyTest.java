package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public abstract class FrameQuaternionReadOnlyTest<F extends FrameQuaternionReadOnly> extends FrameTuple4DReadOnlyTest<F>
{
   public final F createRandom2DFrameTuple(Random random, ReferenceFrame referenceFrame)
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setToYawOrientation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
      return createFrameTuple(referenceFrame, quaternion);
   }

   @Override
   public F createEmptyFrameTuple(ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, new Quaternion());
   }

   @Override
   public final F createRandomFrameTuple(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameTuple(referenceFrame, EuclidCoreRandomTools.nextQuaternion(random));
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameQuaternionReadOnly.class, QuaternionReadOnly.class, true);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameTuple,
                                                                                  methodFilter,
                                                                                  EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Override
   @Test
   public void testEpsilonEquals() throws Exception
   {
      super.testEpsilonEquals();

      Random random = new Random(621541L);
      double epsilon = 0.0;

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.nextReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();
      double s = random.nextDouble();

      F tuple1 = createFrameTuple(frame1, x, y, z, s);
      F tuple2 = createFrameTuple(frame1, x, y, z, s);
      F tuple3 = createFrameTuple(frame2, x, y, z, s);
      F tuple4 = createFrameTuple(frame2, x, y, z, s);

      assertTrue(tuple1.epsilonEquals(tuple2, epsilon));
      assertFalse(tuple1.epsilonEquals(tuple3, epsilon));
      assertFalse(tuple3.epsilonEquals(tuple2, epsilon));
      assertTrue(tuple3.epsilonEquals(tuple4, epsilon));

      assertTrue(tuple1.epsilonEquals((FrameTuple4DReadOnly) tuple2, epsilon));
      assertFalse(tuple1.epsilonEquals((FrameTuple4DReadOnly) tuple3, epsilon));
      assertFalse(tuple3.epsilonEquals((FrameTuple4DReadOnly) tuple2, epsilon));
      assertTrue(tuple3.epsilonEquals((FrameTuple4DReadOnly) tuple4, epsilon));
   }

   @Override
   @Test
   public void testEquals() throws Exception
   {
      super.testEquals();

      Random random = new Random(621541L);

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.nextReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();
      double s = random.nextDouble();

      F tuple1 = createFrameTuple(frame1, x, y, z, s);
      F tuple2 = createFrameTuple(frame1, x, y, z, s);
      F tuple3 = createFrameTuple(frame2, x, y, z, s);
      F tuple4 = createFrameTuple(frame2, x, y, z, s);

      assertTrue(tuple1.equals(tuple2));
      assertFalse(tuple1.equals(tuple3));
      assertFalse(tuple3.equals(tuple2));
      assertTrue(tuple3.equals(tuple4));

      Object tuple2AsObject = tuple2;
      Object tuple3AsObject = tuple3;
      Object tuple4AsObject = tuple4;
      assertTrue(tuple1.equals(tuple2AsObject));
      assertFalse(tuple1.equals(tuple3AsObject));
      assertFalse(tuple3.equals(tuple2AsObject));
      assertTrue(tuple3.equals(tuple4AsObject));
   }

}
