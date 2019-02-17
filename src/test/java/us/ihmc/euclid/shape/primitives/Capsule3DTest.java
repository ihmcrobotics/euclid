package us.ihmc.euclid.shape.primitives;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

class Capsule3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testCommonShape3dFunctionality()
   {
      Shape3DTestHelper testHelper = new Shape3DTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         testHelper.runSimpleTests(EuclidShapeRandomTools.nextCapsule3D(random), random, numberOfPoints);
      }
   }

   @Test
   void testGetSupportingVertex() throws Exception
   {
      Random random = new Random(546161);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Capsule3D capsule = EuclidShapeRandomTools.nextCapsule3D(random);
         Vector3D supportDirection = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly supportingVertex = capsule.getSupportingVertex(supportDirection);
         assertTrue(capsule.isPointInside(supportingVertex));

         Point3D supportingVertexTranslated = new Point3D();
         supportDirection.normalize();
         supportingVertexTranslated.scaleAdd(1.0e-6, supportDirection, supportingVertex);
         assertFalse(capsule.isPointInside(supportingVertexTranslated));

         Vector3D actualNormal = new Vector3D();
         capsule.doPoint3DCollisionTest(supportingVertexTranslated, new Point3D(), actualNormal);
         EuclidCoreTestTools.assertTuple3DEquals(supportDirection, actualNormal, EPSILON);
      }
   }

   @Test
   void testGetBoundingBox() throws Exception
   {
      Random random = new Random(36342);

      for (int i = 0; i < ITERATIONS; i++)
      { // Using getSupportingVertex
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);

         BoundingBox3D expectedBoundingBox = new BoundingBox3D();
         expectedBoundingBox.setToNaN();
         Vector3D supportDirection = new Vector3D(Axis.X);
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis.Y);
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.set(Axis.Z);
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));
         supportDirection.negate();
         expectedBoundingBox.updateToIncludePoint(capsule3D.getSupportingVertex(supportDirection));

         BoundingBox3DReadOnly actualBoundingBox = capsule3D.getBoundingBox();
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expectedBoundingBox, actualBoundingBox, EPSILON);
      }
   }
}
