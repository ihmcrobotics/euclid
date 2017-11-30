package us.ihmc.euclid.referenceFrame;

import org.junit.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

public class FramePose3DTest extends FramePose3DReadOnlyTest<FramePose3D>
{
   @Override
   public FramePose3D createFramePose(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      return new FramePose3D(referenceFrame, pose);
   }
   
   @Test
   public void testConsistencyWithPose3D()
   {
      Random random = new Random(234235L);

      EuclidFrameAPITestTools.FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, pose) -> createFramePose(frame, (Pose3DReadOnly) pose);
      EuclidFrameAPITestTools.GenericTypeBuilder framelessTypeBuilder = () -> createRandomPose(random).getGeometryObject();
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);

      EuclidFrameAPITestTools.GenericTypeBuilder frameless2DTypeBuilder = () -> createRandom2DFramePose(random, ReferenceFrame.getWorldFrame()).getGeometryObject();
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, frameless2DTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Pose3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Pose3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Pose3D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FramePose3D.class, Pose3D.class, true, 1, framelessMethodsToIgnore);
   }
}
