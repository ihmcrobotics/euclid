package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameShapeAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoxPolytope3DView;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameBoxPolytope3DTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testIntegrity()
   {
      Random random = new Random(897234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         FrameBox3D box3D = EuclidFrameShapeRandomTools.nextFrameBox3D(random, worldFrame);
         FrameBoxPolytope3DView boxPolytope = box3D.asConvexPolytope();
         EuclidShapeTestTools.assertConvexPolytope3DGeneralIntegrity("Iteration " + i, boxPolytope);

         assertEquals(6, boxPolytope.getNumberOfFaces());
         assertEquals(12, boxPolytope.getNumberOfEdges());
         assertEquals(8, boxPolytope.getNumberOfVertices());

         for (FrameFace3DReadOnly face : boxPolytope.getFaces())
            assertEquals(4, face.getNumberOfEdges());
         for (FrameVertex3DReadOnly vertex : boxPolytope.getVertices())
            assertEquals(3, vertex.getNumberOfAssociatedEdges());
      }
   }

   @Test
   public void testAgainstConvexPolytope()
   {
      Random random = new Random(987345);
      FrameBox3D box3D = EuclidFrameShapeRandomTools.nextFrameBox3D(random, worldFrame);
      FrameBoxPolytope3DView boxPolytope3D = box3D.asConvexPolytope();

      for (int i = 0; i < ITERATIONS; i++)
      {
         // By re-using the same box, we're ensuring that the polytope view is updating accordingly.
         switch (random.nextInt(3))
         {
            case 0:
               box3D.set(EuclidShapeRandomTools.nextBox3D(random));
               break;
            case 1:
               box3D.getPose().set(EuclidCoreRandomTools.nextRigidBodyTransform(random));
               break;
            case 2:
               box3D.getSize().set(EuclidCoreRandomTools.nextVector3D(random, 0.0, 10.0));
               break;
         }

         FrameConvexPolytope3D convexPolytope3D = new FrameConvexPolytope3D(worldFrame);

         for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         {
            Point3D vertex = new Point3D();
            vertex.set((vertexIndex & 1) == 0 ? box3D.getSizeX() : -box3D.getSizeX(),
                       (vertexIndex & 2) == 0 ? box3D.getSizeY() : -box3D.getSizeY(),
                       (vertexIndex & 4) == 0 ? box3D.getSizeZ() : -box3D.getSizeZ());
            vertex.scale(0.5);
            box3D.transformToWorld(vertex);
            convexPolytope3D.addVertex(vertex);
         }

         assertEquals(boxPolytope3D.getVolume(), convexPolytope3D.getVolume(), EPSILON);
         EuclidCoreTestTools.assertEquals(convexPolytope3D.getCentroid(), boxPolytope3D.getCentroid(), EPSILON);

         for (FrameFace3DReadOnly boxFace : boxPolytope3D.getFaces())
         {
            FrameFace3DReadOnly polytopeFace = convexPolytope3D.getClosestFace(boxFace.getCentroid());

            EuclidFrameTestTools.assertFrameGeometricallyEquals(null, boxFace.getCentroid(), boxFace.getCentroid(), EPSILON);
            EuclidFrameTestTools.assertFrameGeometricallyEquals(null, boxFace.getNormal(), boxFace.getNormal(), EPSILON);

            for (FrameVertex3DReadOnly boxVertex : boxFace.getVertices())
            {
               assertEquals(1L, polytopeFace.getVertices().stream().filter(polytopeVertex -> polytopeVertex.epsilonEquals(boxVertex, EPSILON)).count());
            }
         }
      }
   }

   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());

      {
         List<MethodSignature> signaturesToIgnore = new ArrayList<>();
         signaturesToIgnore.add(new MethodSignature("getSupportingVertex", Vertex3DReadOnly.class, Vector3DReadOnly.class));
         Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
         methodFilter = methodFilter.and(m -> m.getParameterTypes().equals(new Class<?>[] {Axis3D.class}));
         tester.assertOverloadingWithFrameObjects(FrameBoxPolytope3DView.class, BoxPolytope3DView.class, false, 1, methodFilter);
      }
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("getBoundingBox", FixedFrameBoundingBox3DBasics.class));
      signaturesToIgnore.add(new MethodSignature("getBoundingBox", ReferenceFrame.class, FrameBoundingBox3DBasics.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      methodFilter = methodFilter.and(m -> Stream.of(m.getParameterTypes()).noneMatch(type -> Vertex3DReadOnly.class.isAssignableFrom(type)));
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(FrameBoxPolytope3DTest::nextFrameBoxPolytope3DView,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithBox3D()
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("hashCode"));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", FrameConvexPolytope3D.class, double.class));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", FrameConvexPolytope3D.class, double.class));
      signaturesToIgnore.add(new MethodSignature("set", FrameConvexPolytope3D.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      methodFilter = methodFilter.and(m -> Stream.of(m.getParameterTypes()).noneMatch(type -> Vertex3DReadOnly.class.isAssignableFrom(type)));
      methodFilter = methodFilter.and(m -> m.getName().equals("othogonalProjectionCopy"));
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameShapeAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality((frame, boxPolytope) -> copy(frame, (BoxPolytope3DView) boxPolytope),
                                                                  FrameBoxPolytope3DTest::nextBoxPolytope3DView,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   /**
    * Test exposing a bug about the centroid of the faces for a frame box.
    * <p>
    * The bug was due to a bad location for clearing a dirty flag. By moving the clearing to before
    * actually calculation, this would prevent the centroid to be recomputed several time.
    * </p>
    */
   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testFaceCentroidBug()
   {
      Box3D unitBox3D = new Box3D(1.0, 1.0, 1.0);
      FrameBox3D unitFrameBox3D = new FrameBox3D(worldFrame, 1.0, 1.0, 1.0);
      BoxPolytope3DView boxPolytope = unitBox3D.asConvexPolytope();
      FrameBoxPolytope3DView frameBoxPolytope = unitFrameBox3D.asConvexPolytope();

      assertEquals(new Point3D(-0.5, 0.0, 0.0), boxPolytope.getFace(0).getCentroid());
      assertEquals(new Point3D(0.0, -0.5, 0.0), boxPolytope.getFace(1).getCentroid());
      assertEquals(new Point3D(0.0, 0.0, -0.5), boxPolytope.getFace(2).getCentroid());
      assertEquals(new Point3D(0.5, 0.0, 0.0), boxPolytope.getFace(3).getCentroid());
      assertEquals(new Point3D(0.0, 0.5, 0.0), boxPolytope.getFace(4).getCentroid());
      assertEquals(new Point3D(0.0, 0.0, 0.5), boxPolytope.getFace(5).getCentroid());

      assertEquals(new Point3D(-0.5, 0.0, 0.0), frameBoxPolytope.getFace(0).getCentroid());
      assertEquals(new Point3D(0.0, -0.5, 0.0), frameBoxPolytope.getFace(1).getCentroid());
      assertEquals(new Point3D(0.0, 0.0, -0.5), frameBoxPolytope.getFace(2).getCentroid());
      assertEquals(new Point3D(0.5, 0.0, 0.0), frameBoxPolytope.getFace(3).getCentroid());
      assertEquals(new Point3D(0.0, 0.5, 0.0), frameBoxPolytope.getFace(4).getCentroid());
      assertEquals(new Point3D(0.0, 0.0, 0.5), frameBoxPolytope.getFace(5).getCentroid());

      for (int i = 0; i < 8; i++)
      {
         assertTrue(frameBoxPolytope.getVertices().contains(unitFrameBox3D.getVertex(i)));
         assertEquals(boxPolytope.getVertex(i), frameBoxPolytope.getVertex(i));
         assertEquals(boxPolytope.getHalfEdge(i), frameBoxPolytope.getHalfEdge(i));
         assertEquals(1.0, boxPolytope.getHalfEdge(i).length());
         assertEquals(1.0, frameBoxPolytope.getHalfEdge(i).length());
      }

      for (int i = 0; i < 6; i++)
      {
         Face3DReadOnly boxFace = boxPolytope.getFace(i);
         FrameFace3DReadOnly frameBoxFace = frameBoxPolytope.getFace(i);

         for (int j = 0; j < 4; j++)
         {
            assertEquals(boxFace.getVertex(j), frameBoxFace.getVertex(j));
         }

         assertEquals(boxFace.getNormal(), frameBoxFace.getNormal());
         assertEquals(boxFace.getCentroid(), frameBoxFace.getCentroid());
         assertEquals(boxFace.getBoundingBox(), frameBoxFace.getBoundingBox());
         assertEquals(1.0, boxFace.getArea());
         assertEquals(1.0, frameBoxFace.getArea());
      }
   }

   private static BoxPolytope3DView nextBoxPolytope3DView(Random random)
   {
      return EuclidShapeRandomTools.nextBox3D(random).asConvexPolytope();
   }

   private static FrameBoxPolytope3DView nextFrameBoxPolytope3DView(Random random, ReferenceFrame referenceFrame)
   {
      return EuclidFrameShapeRandomTools.nextFrameBox3D(random, referenceFrame).asConvexPolytope();
   }

   private static FrameBoxPolytope3DView copy(ReferenceFrame referenceFrame, BoxPolytope3DView boxPolytope3DView)
   {
      return new FrameBox3D(referenceFrame, (Box3DReadOnly) boxPolytope3DView.copy()).asConvexPolytope();
   }
}
