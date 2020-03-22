package us.ihmc.euclid.referenceFrame;

import org.junit.jupiter.api.BeforeAll;

import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.ReflectionBasedBuilders;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;

public class FrameShapeSetupTest
{
   @BeforeAll
   public static void registerFrameShape()
   {
      ReflectionBasedBuilders.registerRandomGeneratorClasses(EuclidFrameShapeRandomTools.class, EuclidShapeRandomTools.class);
      EuclidFrameAPITester.registerFramelessTypesSmart(Torus3DBasics.class);
      EuclidFrameAPITester.registerFrameTypesSmart(FrameBox3DBasics.class,
                                                   FrameCapsule3DBasics.class,
                                                   FrameCylinder3DBasics.class,
                                                   FrameEllipsoid3DBasics.class,
                                                   FramePointShape3DBasics.class,
                                                   FrameRamp3DBasics.class,
                                                   FrameShape3DPoseBasics.class,
                                                   FrameSphere3DBasics.class);
   }
}
