package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Tuple4DReadOnlyTest;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public abstract class FrameQuaternionReadOnlyTest<T extends FrameQuaternionReadOnly> extends Tuple4DReadOnlyTest<T>
{
   public abstract T createFrameQuaternion(ReferenceFrame referenceFrame, QuaternionReadOnly quaternion);

   public abstract T createFrameQuaternion(ReferenceFrame referenceFrame, double x, double y, double z, double s);

   @Override
   public final T createEmptyTuple()
   {
      return createEmptyFrameQuaternion(ReferenceFrame.getWorldFrame());
   }

   @Override
   public final T createRandomTuple(Random random)
   {
      return createRandomFrameQuaternion(random, ReferenceFrame.getWorldFrame());
   }

   @Override
   public final T createTuple(double x, double y, double z, double s)
   {
      return createFrameQuaternion(ReferenceFrame.getWorldFrame(), x, y, z, s);
   }

   public final T createEmptyFrameQuaternion(ReferenceFrame referenceFrame)
   {
      return createFrameQuaternion(referenceFrame, new Quaternion());
   }

   public final T createRandomFrameQuaternion(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameQuaternion(referenceFrame, EuclidCoreRandomTools.generateRandomQuaternion(random));
   }

   public final T createRandom2DFrameQuaternion(Random random, ReferenceFrame referenceFrame)
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setToYawQuaternion(EuclidCoreRandomTools.generateRandomDouble(random, Math.PI));
      return createFrameQuaternion(referenceFrame, quaternion);
   }

   @Override
   public double getEpsilon()
   {
      return 1e-10;
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameQuaternionReadOnly.class, QuaternionReadOnly.class, true);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Random random = new Random(234);
      Predicate<Method> methodFilter = m -> !m.getName().contains("IncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame -> createRandomFrameQuaternion(random, frame), false, true, methodFilter);
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame -> createRandom2DFrameQuaternion(random, frame), false, true, methodFilter);
   }
}
