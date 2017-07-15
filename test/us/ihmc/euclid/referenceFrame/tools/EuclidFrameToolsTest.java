package us.ihmc.euclid.referenceFrame.tools;

import java.util.HashMap;
import java.util.Map;

import org.junit.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EuclidFrameToolsTest
{
   private static final Class<?> d = Double.TYPE;

   @Test
   public void testAPIIsComplete()
   {
      Map<String, Class<?>[]> methodsToIgnore = new HashMap<>();
      methodsToIgnore.put("orthogonalProjectionOnLine3D", new Class<?>[] {Point3DReadOnly.class, d, d, d, d, d, d, Point3DBasics.class});
      methodsToIgnore.put("orthogonalProjectionOnLine2D", new Class<?>[] {Point2DReadOnly.class, d, d, d, d, Point2DBasics.class});
      methodsToIgnore.put("orthogonalProjectionOnLineSegment2D", new Class<?>[] {Point2DReadOnly.class, d, d, d, d, Point2DBasics.class});
      methodsToIgnore.put("orthogonalProjectionOnLineSegment3D", new Class<?>[] {Point3DReadOnly.class, d, d, d, d, d, d, Point3DBasics.class});
      methodsToIgnore.put("intersectionBetweenLine3DAndBoundingBox3D", new Class<?>[] {d, d, d, d, d, d, d, d, d, d, d, d, Point3DBasics.class, Point3DBasics.class});
      methodsToIgnore.put("intersectionBetweenLine3DAndCylinder3D", new Class<?>[] {d, d, d, d, d, d, d, d, Point3DBasics.class, Point3DBasics.class});
      methodsToIgnore.put("intersectionBetweenLine3DAndEllipsoid3D", new Class<?>[] {d, d, d, d, d, d, d, d, d, Point3DBasics.class, Point3DBasics.class});

      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(EuclidFrameTools.class, EuclidGeometryTools.class, false, false, 2, methodsToIgnore);
   }
}
