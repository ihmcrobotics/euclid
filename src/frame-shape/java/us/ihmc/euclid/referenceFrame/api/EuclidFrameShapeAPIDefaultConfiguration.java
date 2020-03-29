package us.ihmc.euclid.referenceFrame.api;

import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DBasics;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.referenceFrame.polytope.FrameFace3D;
import us.ihmc.euclid.referenceFrame.polytope.FrameHalfEdge3D;
import us.ihmc.euclid.referenceFrame.polytope.FrameVertex3D;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameHalfEdge3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;

public class EuclidFrameShapeAPIDefaultConfiguration extends EuclidFrameAPIDefaultConfiguration
{
   @Override
   public void configure(EuclidFrameAPITester testerToConfigure, ReflectionBasedBuilder builderToConfigure)
   {
      super.configure(testerToConfigure, builderToConfigure);

      builderToConfigure.registerRandomGeneratorClasses(EuclidFrameShapeRandomTools.class, EuclidShapeRandomTools.class);
      testerToConfigure.registerFramelessTypesSmart(Torus3DBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameBox3DBasics.class,
                                                FrameCapsule3DBasics.class,
                                                FrameCylinder3DBasics.class,
                                                FrameEllipsoid3DBasics.class,
                                                FramePointShape3DBasics.class,
                                                FrameRamp3DBasics.class,
                                                FrameShape3DPoseBasics.class,
                                                FrameSphere3DBasics.class);
      testerToConfigure.registerFrameType(FrameConvexPolytope3D.class,
                                          null,
                                          FrameConvexPolytope3DReadOnly.class,
                                          ConvexPolytope3D.class,
                                          ConvexPolytope3DReadOnly.class);
      testerToConfigure.registerFrameType(null, FrameFace3D.class, FrameFace3DReadOnly.class, Face3D.class, Face3DReadOnly.class);
      testerToConfigure.registerFrameType(null, FrameHalfEdge3D.class, FrameHalfEdge3DReadOnly.class, HalfEdge3D.class, HalfEdge3DReadOnly.class);
      testerToConfigure.registerFrameType(null, FrameVertex3D.class, FrameVertex3DReadOnly.class, Vertex3D.class, Vertex3DReadOnly.class);

   }
}
