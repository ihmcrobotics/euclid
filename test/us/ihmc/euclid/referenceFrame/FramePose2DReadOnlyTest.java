package us.ihmc.euclid.referenceFrame;

import org.junit.Test;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

import java.util.Random;

public abstract class FramePose2DReadOnlyTest<T extends FramePose2DReadOnly>
{
   public abstract T createFramePose(ReferenceFrame referenceFrame, Pose2DReadOnly pose);

   public final T createEmptyFramePose(ReferenceFrame referenceFrame)
   {
      return createFramePose(referenceFrame, new Pose2D());
   }

   public final T createRandomPose(Random random)
   {
      return createRandomFramePose(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFramePose(Random random, ReferenceFrame referenceFrame)
   {
      return createFramePose(referenceFrame, EuclidGeometryRandomTools.nextPose2D(random));
   }

   public final T createRandom2DFramePose(Random random, ReferenceFrame referenceFrame)
   {
      Pose2D pose = new Pose2D();
      pose.setYaw(EuclidCoreRandomTools.nextDouble(random, Math.PI));
      pose.setPosition(EuclidCoreRandomTools.nextPoint2D(random));
      return createFramePose(referenceFrame, pose);
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FramePose2DReadOnly.class, Pose2DReadOnly.class, true);
   }
}
