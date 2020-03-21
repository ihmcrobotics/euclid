package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public abstract class FrameMatrix3DReadOnlyTest<T extends FrameMatrix3DReadOnly>
{
   public abstract T createFrameMatrix3D(ReferenceFrame referenceFrame, Matrix3DReadOnly pose);

   public final T createEmptyFrameMatrix3D(ReferenceFrame referenceFrame)
   {
      return createFrameMatrix3D(referenceFrame, new Matrix3D());
   }

   public final T createRandomFrameMatrix3D(Random random)
   {
      return createRandomFrameMatrix3D(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFrameMatrix3D(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameMatrix3D(referenceFrame, EuclidCoreRandomTools.nextMatrix3D(random));
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().contains("IncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals")
            && !m.getName().equals("setMatchingFrame");
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameMatrix3D,
                                                                                  methodFilter,
                                                                                  EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FramePose2DReadOnly.class, Pose2DReadOnly.class, true);
   }
}
