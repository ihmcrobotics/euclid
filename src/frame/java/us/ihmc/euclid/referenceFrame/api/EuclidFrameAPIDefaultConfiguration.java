package us.ihmc.euclid.referenceFrame.api;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector4DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

/**
 * Default API tester configuration that includes the types of the {@code euclid-frame} project.
 * <p>
 * This interface is part of the API testing framework.
 * </p>
 *
 * @see EuclidFrameAPITester
 * @author Sylvain Bertrand
 */
public class EuclidFrameAPIDefaultConfiguration implements EuclidFrameAPITestConfiguration
{
   @Override
   public void configure(EuclidFrameAPITester testerToConfigure, ReflectionBasedBuilder builderToConfigure)
   {
      builderToConfigure.registerRandomGeneratorClasses(EuclidCoreRandomTools.class, EuclidGeometryRandomTools.class, EuclidFrameRandomTools.class);

      testerToConfigure.registerExceptionsToIgnore(BoundingBoxException.class, IllegalArgumentException.class, RuntimeException.class);
      testerToConfigure.registerFrameTypesSmart(FrameTuple2DBasics.class, FrameVector2DBasics.class, FramePoint2DBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameTuple3DBasics.class, FrameVector3DBasics.class, FramePoint3DBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameTuple4DBasics.class, FrameVector4DBasics.class, FrameQuaternionBasics.class);
      testerToConfigure.registerFrameTypeSmart(FrameYawPitchRollBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameRotationMatrixBasics.class, FrameMatrix3DBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameOrientation2DBasics.class, FrameOrientation3DBasics.class);
      testerToConfigure.registerFrameTypesSmart(FramePose2DBasics.class, FramePose3DBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameLine2DBasics.class, FrameLine3DBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameLineSegment2DBasics.class, FrameLineSegment3DBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameConvexPolygon2DBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameBoundingBox2DBasics.class, FrameBoundingBox3DBasics.class);

      testerToConfigure.registerReadOnlyFrameTypeSmart(FrameVertex2DSupplier.class);
      testerToConfigure.registerReadOnlyFrameTypeSmart(FrameVertex3DSupplier.class);

      testerToConfigure.registerFramelessTypeSmart(AxisAngleBasics.class);
      testerToConfigure.registerFramelessType(RotationScaleMatrix.class, RotationScaleMatrixReadOnly.class);
   }
}
