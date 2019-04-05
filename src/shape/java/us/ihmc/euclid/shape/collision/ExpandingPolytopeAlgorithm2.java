package us.ihmc.euclid.shape.collision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class ExpandingPolytopeAlgorithm2
{
   private static final boolean VERBOSE = false;

   private double epsilon = 1.0e-12;
   private int numberOfIterations = 0;
   private int maxIterations = 10000000;
   private final GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private boolean latestCollisionTestResult;
   private EPAFace3D lastResult = null;

   public ExpandingPolytopeAlgorithm2()
   {
   }

   public Shape3DCollisionTestResult doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      doCollisionTest(shapeA, shapeB);
      Shape3DCollisionTestResult result = new Shape3DCollisionTestResult();
      if (gjkCollisionDetector.getSimplexVertices() == null)
         return null;
      result.setToNaN();
      packResult(shapeA, shapeB, result);
      return result;
   }

   public void doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, Shape3DCollisionTestResult result)
   {
      doCollisionTest(shapeA, shapeB);
      result.setToNaN();

      if (gjkCollisionDetector.getSimplexVertices() == null)
         return;

      packResult(shapeA, shapeB, result);
   }

   public boolean doCollisionTest(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB)
   {
      latestCollisionTestResult = gjkCollisionDetector.doCollisionTest(shapeA, shapeB);
      if (latestCollisionTestResult)
         doCollision(shapeA, shapeB, gjkCollisionDetector.getSimplexVertices());
      return latestCollisionTestResult;
   }

   public void doCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, DifferenceVertex3D[] simplex)
   {
      PriorityQueue<EPAFace3D> queue = new PriorityQueue<>();
      double mu = Double.POSITIVE_INFINITY;

      List<EPAFace3D> initialPolytope = fromDifferenceVertices(shapeA, shapeB, simplex, epsilon);
      initialPolytope.stream().filter(EPAFace3D::isClosestPointInternal).forEach(queue::add);
      Vector3D supportDirection = new Vector3D();

      for (int i = 0; i < maxIterations; i++)
      {
         numberOfIterations = i;

         if (queue.isEmpty())
         {
            if (VERBOSE)
               System.out.println("Queue is empty, terminating.");
            break;
         }

         EPAFace3D entry = queue.poll();
         if (entry.isDestroyed())
            continue;
         double currentNormSquared = entry.normSquared;

         if (currentNormSquared > mu)
         {
            if (VERBOSE)
               System.out.println("Best norm exceeds upper bound, terminating.");
            break;
         }

         lastResult = entry;

         if (currentNormSquared > epsilon)
            supportDirection.set(entry.closestPoint);
         else
            supportDirection.set(entry.getNormal());
         Point3DReadOnly vertexA = shapeA.getSupportingVertex(supportDirection);
         supportDirection.negate();
         Point3DReadOnly vertexB = shapeB.getSupportingVertex(supportDirection);
         EPAVertex3D newVertex = new EPAVertex3D(vertexA, vertexB);

         if (entry.contains(newVertex))
         {
            if (VERBOSE)
               System.out.println("New vertex equals to an vertex of entry, terminating.");
            break;
         }

         mu = Math.min(mu, square(TupleTools.dot(newVertex, supportDirection)) / currentNormSquared);

         if (mu <= square(1.0 + epsilon) * currentNormSquared)
         {
            if (VERBOSE)
               System.out.println("Reached max accuracy, terminating.");
            break;
         }

         entry.destroy();
         List<EPAEdge3D> silhouette = new ArrayList<>();
         silhouette(entry.e0.twin, newVertex, silhouette);
         silhouette(entry.e1.twin, newVertex, silhouette);
         silhouette(entry.e2.twin, newVertex, silhouette);

         boolean areNewTrianglesFine = true;

         for (EPAEdge3D sentryEdge : silhouette)
         {
            EPAFace3D newEntry = EPAFace3D.fromVertexAndTwinEdge(newVertex, sentryEdge, epsilon);

            if (newEntry.isTriangleAffinelyDependent)
            {
               if (VERBOSE)
                  System.out.println("New triangle affinely dependent, terminating.");
               areNewTrianglesFine = false;
               break;
            }
            if (newEntry.isClosestPointInternal && currentNormSquared <= newEntry.normSquared && newEntry.normSquared <= mu)
               queue.add(newEntry);
         }

         if (!areNewTrianglesFine)
            break;

         boolean malformedSilhouette = false;

         for (int edgeIndex = 0; edgeIndex < newVertex.getNumberOfAssociatedEdges(); edgeIndex++)
         {
            EPAEdge3D edge = newVertex.getAssociatedEdge(edgeIndex);
            EPAEdge3D twin = edge.v1.getEdgeTo(newVertex);

            if (twin == null)
            {
               if (VERBOSE)
                  System.out.println("Silhouette is malformed, terminating.");
               malformedSilhouette = true;
               break;
            }

            edge.setTwin(twin);
         }

         if (malformedSilhouette)
         {
            break;
         }
      }

      if (VERBOSE)
         System.out.println("Number of iterations: " + numberOfIterations);
   }

   private void packResult(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, Shape3DCollisionTestResult result)
   {
      if (latestCollisionTestResult)
      {
         result.setToNaN();
         result.setShapesAreColliding(latestCollisionTestResult);
         result.setShapeA(shapeA);
         result.setShapeB(shapeB);
         result.setDistance(latestCollisionTestResult ? -lastResult.getDistanceFromOrigin() : gjkCollisionDetector.getDistance());

         result.getPointOnA().set(lastResult.computePointOnA());
         result.getPointOnB().set(lastResult.computePointOnB());
      }
      else
      {
         gjkCollisionDetector.packResult(shapeA, shapeB, result, false);
      }
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }

   public ConvexPolytope3DReadOnly getPolytope()
   {
      return new EPAConvexPolytope3D(lastResult);
   }

   private static double square(double value)
   {
      return value * value;
   }

   private static void silhouette(EPAEdge3D edge, Point3DReadOnly observer, List<EPAEdge3D> silhouetteToPack)
   {
      if (edge == null || edge.face.isDestroyed())
         return;

      if (TupleTools.dot(edge.face.closestPoint, observer) < edge.face.closestPoint.distanceFromOriginSquared())
      {
         silhouetteToPack.add(edge);
      }
      else
      {
         edge.face.destroy();
         silhouette(edge.next.twin, observer, silhouetteToPack);
         silhouette(edge.prev.twin, observer, silhouetteToPack);
      }
   }

   private static class EPAConvexPolytope3D implements ConvexPolytope3DReadOnly
   {
      private final List<EPAFace3D> faces = new ArrayList<>();
      private final List<EPAEdge3D> halfEdges = new ArrayList<>();
      private final List<EPAVertex3D> vertices = new ArrayList<>();

      private final double volume;
      private final Point3D centroid = new Point3D();

      public EPAConvexPolytope3D(EPAVertex3D startVertex)
      {
         Set<EPAFace3D> faceSet = new HashSet<>();
         collectFacesRecursively(startVertex.getAssociatedEdge(0).getFace(), faceSet);
         faces.addAll(faceSet);

         for (EPAFace3D face : faceSet)
         {
            halfEdges.add(face.e0);
            halfEdges.add(face.e1);
            halfEdges.add(face.e2);
         }

         halfEdges.stream().map(EPAEdge3D::getOrigin).distinct().forEach(vertices::add);
         volume = EuclidPolytopeConstructionTools.computeConvexPolytope3DVolume(this, centroid);
      }

      public EPAConvexPolytope3D(EPAFace3D startFace)
      {
         Set<EPAFace3D> faceSet = new HashSet<>();
         collectFacesRecursively(startFace, faceSet);
         faces.addAll(faceSet);

         for (EPAFace3D face : faceSet)
         {
            halfEdges.add(face.e0);
            halfEdges.add(face.e1);
            halfEdges.add(face.e2);
         }

         halfEdges.stream().map(EPAEdge3D::getOrigin).distinct().forEach(vertices::add);
         volume = EuclidPolytopeConstructionTools.computeConvexPolytope3DVolume(this, centroid);
      }

      private static void collectFacesRecursively(EPAFace3D face, Set<EPAFace3D> facesToPack)
      {
         if (face.isDestroyed() || !facesToPack.add(face))
            return;

         collectFacesRecursively(face.e0.twin.face, facesToPack);
         collectFacesRecursively(face.e1.twin.face, facesToPack);
         collectFacesRecursively(face.e2.twin.face, facesToPack);
      }

      @Override
      public BoundingBox3DReadOnly getBoundingBox()
      {
         BoundingBox3D boundingBox3D = new BoundingBox3D();
         boundingBox3D.setToNaN();
         faces.forEach(face -> boundingBox3D.combine(face.getBoundingBox()));
         return boundingBox3D;
      }

      @Override
      public Point3DReadOnly getCentroid()
      {
         return centroid;
      }

      @Override
      public double getVolume()
      {
         return volume;
      }

      @Override
      public double getConstructionEpsilon()
      {
         return 0;
      }

      @Override
      public List<? extends Face3DReadOnly> getFaces()
      {
         return faces;
      }

      @Override
      public List<? extends Vertex3DReadOnly> getVertices()
      {
         return vertices;
      }

      @Override
      public List<? extends HalfEdge3DReadOnly> getHalfEdges()
      {
         return halfEdges;
      }

      @Override
      public String toString()
      {
         return EuclidShapeIOTools.getConvexPolytope3DString(this);
      }
   }

   private static class EPAFace3D implements Comparable<EPAFace3D>, Face3DReadOnly
   {
      private final EPAVertex3D v0, v1, v2;
      private final EPAEdge3D e0, e1, e2;
      private boolean destroyed = false;

      private final Point3DReadOnly closestPoint;
      private final double lambda0, lambda1, lambda2;
      private final boolean isTriangleAffinelyDependent;
      private final boolean isClosestPointInternal;
      private final double normSquared;
      private double norm = Double.NaN;

      public static EPAFace3D fromVertexAndTwinEdge(EPAVertex3D vertex, EPAEdge3D twin, double epsilon)
      {
         EPAVertex3D v0 = twin.v1;
         EPAVertex3D v1 = twin.v0;
         EPAVertex3D v2 = vertex;
         EPAFace3D face = new EPAFace3D(v0, v1, v2, epsilon);
         face.e0.setTwin(twin);
         return face;
      }

      public EPAFace3D(EPAVertex3D v0, EPAVertex3D v1, EPAVertex3D v2, double epsilon)
      {
         this.v0 = v0;
         this.v1 = v1;
         this.v2 = v2;
         e0 = new EPAEdge3D(this, v0, v1);
         e1 = new EPAEdge3D(this, v1, v2);
         e2 = new EPAEdge3D(this, v2, v0);
         e0.next = e1;
         e1.next = e2;
         e2.next = e0;

         e0.prev = e2;
         e1.prev = e0;
         e2.prev = e1;

         double[] lambdas = signedVolumeFor2Simplex(v0, v1, v2, epsilon);
         isTriangleAffinelyDependent = lambdas == null;

         if (!isTriangleAffinelyDependent)
         {
            lambda0 = lambdas[0];
            lambda1 = lambdas[1];
            lambda2 = lambdas[2];

            isClosestPointInternal = lambda0 >= 0.0 && lambda1 >= 0.0 && lambda2 >= 0.0;

            Point3D point = new Point3D();
            point.setAndScale(lambda0, v0);
            point.scaleAdd(lambda1, v1, point);
            point.scaleAdd(lambda2, v2, point);
            closestPoint = point;
            normSquared = closestPoint.distanceFromOriginSquared();
         }
         else
         {
            lambda0 = Double.NaN;
            lambda1 = Double.NaN;
            lambda2 = Double.NaN;
            isClosestPointInternal = false;
            closestPoint = null;
            normSquared = Double.NaN;
         }
      }

      public boolean contains(Point3DReadOnly query)
      {
         return v0.equals(query) || v1.equals(query) || v2.equals(query);
      }

      public double getDistanceFromOrigin()
      {
         if (Double.isNaN(norm))
            norm = Math.sqrt(normSquared);
         return norm;
      }

      public void destroy()
      {
         destroyed = true;
         e0.destroy();
         e1.destroy();
         e2.destroy();
      }

      public boolean isDestroyed()
      {
         return destroyed;
      }

      public boolean isClosestPointInternal()
      {
         return isClosestPointInternal;
      }

      public Point3D computePointOnA()
      {
         Point3D pointOnA = new Point3D();
         pointOnA.scaleAdd(lambda0, v0.getVertexOnShapeA(), pointOnA);
         pointOnA.scaleAdd(lambda1, v1.getVertexOnShapeA(), pointOnA);
         pointOnA.scaleAdd(lambda2, v2.getVertexOnShapeA(), pointOnA);
         return pointOnA;
      }

      public Point3D computePointOnB()
      {
         Point3D pointOnB = new Point3D();
         pointOnB.scaleAdd(lambda0, v0.getVertexOnShapeB(), pointOnB);
         pointOnB.scaleAdd(lambda1, v1.getVertexOnShapeB(), pointOnB);
         pointOnB.scaleAdd(lambda2, v2.getVertexOnShapeB(), pointOnB);
         return pointOnB;
      }

      @Override
      public int compareTo(EPAFace3D other)
      {
         if (normSquared == other.normSquared)
            return 0;
         if (normSquared > other.normSquared)
            return 1;
         return -1;
      }

      @Override
      public double getArea()
      {
         return EuclidGeometryTools.triangleArea(v0, v1, v2);
      }

      @Override
      public BoundingBox3DReadOnly getBoundingBox()
      {
         BoundingBox3D boundingBox3D = new BoundingBox3D();
         boundingBox3D.setToNaN();
         boundingBox3D.updateToIncludePoint(v0);
         boundingBox3D.updateToIncludePoint(v1);
         boundingBox3D.updateToIncludePoint(v2);
         return boundingBox3D;
      }

      @Override
      public Point3DReadOnly getCentroid()
      {
         return EuclidGeometryTools.averagePoint3Ds(Arrays.asList(v0, v1, v2));
      }

      @Override
      public List<? extends HalfEdge3DReadOnly> getEdges()
      {
         return Arrays.asList(e0, e1, e2);
      }

      @Override
      public Vector3DReadOnly getNormal()
      {
         return EuclidPolytopeTools.crossProductOfLineSegment3Ds(v1, v0, v1, v2);
      }

      @Override
      public String toString()
      {
         return EuclidShapeIOTools.getFace3DString(this);
      }
   }

   private static class EPAEdge3D implements HalfEdge3DReadOnly
   {
      private final EPAFace3D face;
      private final EPAVertex3D v0, v1;
      private EPAEdge3D twin;
      private EPAEdge3D next;
      private EPAEdge3D prev;

      public EPAEdge3D(EPAFace3D face, EPAVertex3D v0, EPAVertex3D v1)
      {
         this.face = face;
         this.v0 = v0;
         this.v1 = v1;
         v0.addAssociatedEdge(this);
      }

      public void setTwin(EPAEdge3D twin)
      {
         if (v0 != twin.v1 || v1 != twin.v0)
            throw new IllegalArgumentException("Twin does not match: this edge:\n" + this + "\ntwin:\n" + twin);

         this.twin = twin;

         if (twin.twin != this)
            twin.setTwin(this);
      }

      public void destroy()
      {
         v0.removeAssociatedEdge(this);
      }

      @Override
      public EPAVertex3D getOrigin()
      {
         return v0;
      }

      @Override
      public EPAVertex3D getDestination()
      {
         return v1;
      }

      @Override
      public EPAEdge3D getNext()
      {
         return next;
      }

      @Override
      public EPAEdge3D getPrevious()
      {
         return prev;
      }

      @Override
      public EPAEdge3D getTwin()
      {
         return twin;
      }

      @Override
      public EPAFace3D getFace()
      {
         return face;
      }

      @Override
      public String toString()
      {
         return EuclidShapeIOTools.getHalfEdge3DString(this);
      }
   }

   private static class EPAVertex3D implements Vertex3DReadOnly
   {
      private final double x, y, z;
      private final Point3DReadOnly vertexOnShapeA;
      private final Point3DReadOnly vertexOnShapeB;
      private final List<EPAEdge3D> associatedEdges = new ArrayList<>();

      public EPAVertex3D(Point3DReadOnly vertexOnShapeA, Point3DReadOnly vertexOnShapeB)
      {
         this.vertexOnShapeA = vertexOnShapeA;
         this.vertexOnShapeB = vertexOnShapeB;
         x = vertexOnShapeA.getX() - vertexOnShapeB.getX();
         y = vertexOnShapeA.getY() - vertexOnShapeB.getY();
         z = vertexOnShapeA.getZ() - vertexOnShapeB.getZ();
      }

      public void addAssociatedEdge(EPAEdge3D edge)
      {
         if (!isAssociatedWithEdge(edge))
         {
            if (edge.v0 != this)
               throw new IllegalArgumentException("A vertex's associated edges should originate from this same vertex.");
            associatedEdges.add(edge);
         }
      }

      public void removeAssociatedEdge(EPAEdge3D edgeToAdd)
      {
         associatedEdges.remove(edgeToAdd);
      }

      @Override
      public List<EPAEdge3D> getAssociatedEdges()
      {
         return associatedEdges;
      }

      @Override
      public EPAEdge3D getAssociatedEdge(int index)
      {
         return associatedEdges.get(index);
      }

      @Override
      public int getNumberOfAssociatedEdges()
      {
         return associatedEdges.size();
      }

      @Override
      public EPAEdge3D getEdgeTo(Vertex3DReadOnly destination)
      {
         return (EPAEdge3D) Vertex3DReadOnly.super.getEdgeTo(destination);
      }

      public Point3DReadOnly getVertexOnShapeA()
      {
         return vertexOnShapeA;
      }

      public Point3DReadOnly getVertexOnShapeB()
      {
         return vertexOnShapeB;
      }

      @Override
      public double getX()
      {
         return x;
      }

      @Override
      public double getY()
      {
         return y;
      }

      @Override
      public double getZ()
      {
         return z;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
            return true;
         else if (object instanceof Point3DReadOnly)
            return equals((Point3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return EuclidShapeIOTools.getVertex3DString(this);
      }
   }

   public static double[] signedVolumeFor2Simplex(Point3DReadOnly s1, Point3DReadOnly s2, Point3DReadOnly s3, double epsilon)
   {
      double s1x3D = s1.getX(), s1y3D = s1.getY(), s1z3D = s1.getZ();
      double s2x3D = s2.getX(), s2y3D = s2.getY(), s2z3D = s2.getZ();
      double s3x3D = s3.getX(), s3y3D = s3.getY(), s3z3D = s3.getZ();

      double s2s1x3D = s1x3D - s2x3D;
      double s2s1y3D = s1y3D - s2y3D;
      double s2s1z3D = s1z3D - s2z3D;

      double s3s1x3D = s1x3D - s3x3D;
      double s3s1y3D = s1y3D - s3y3D;
      double s3s1z3D = s1z3D - s3z3D;

      // Computing the face's normal as n = (s2 - s1) x (s3 - s1)
      double nx = s2s1y3D * s3s1z3D - s2s1z3D * s3s1y3D;
      double ny = s2s1z3D * s3s1x3D - s2s1x3D * s3s1z3D;
      double nz = s2s1x3D * s3s1y3D - s2s1y3D * s3s1x3D;

      // Computing the projection p0 of the origin (0, 0, 0) onto the face as p0 = (s1.n) / (n.n) n
      double distanceFromPlane = TupleTools.dot(nx, ny, nz, s1) / EuclidCoreTools.normSquared(nx, ny, nz);
      double p0x3D = distanceFromPlane * nx;
      double p0y3D = distanceFromPlane * ny;
      double p0z3D = distanceFromPlane * nz;

      double muMax = 0.0; // mu = area of the triangle (s1, s2, s3)
      double s1x, s2x, s3x, p0x;
      double s1y, s2y, s3y, p0y;

      // Finding muMax and the coordinates that generate it.
      // As described in the paper, the 3D triangle is projected on each Cartesian plane. The projected 2D triangle with the largest area is selected to pursue further computation.
      double mu;

      // Area of the triangle (s1, s2, s3) projected onto the YZ-plane.
      mu = s1y3D * (s2z3D - s3z3D) + s2y3D * (s3z3D - s1z3D) + s3y3D * (s1z3D - s2z3D);

      muMax = mu;
      s1x = s1y3D;
      s2x = s2y3D;
      s3x = s3y3D;
      p0x = p0y3D;
      s1y = s1z3D;
      s2y = s2z3D;
      s3y = s3z3D;
      p0y = p0z3D;

      // Area of the triangle (s1, s2, s3) projected onto the XZ-plane.
      mu = s1z3D * (s2x3D - s3x3D) + s2z3D * (s3x3D - s1x3D) + s3z3D * (s1x3D - s2x3D);

      if (Math.abs(mu) > Math.abs(muMax))
      {
         muMax = mu;
         s1x = s1z3D;
         s2x = s2z3D;
         s3x = s3z3D;
         p0x = p0z3D;
         s1y = s1x3D;
         s2y = s2x3D;
         s3y = s3x3D;
         p0y = p0x3D;
      }

      // Area of the triangle (s1, s2, s3) projected onto the XY-plane.
      mu = s1x3D * (s2y3D - s3y3D) + s2x3D * (s3y3D - s1y3D) + s3x3D * (s1y3D - s2y3D);

      if (Math.abs(mu) > Math.abs(muMax))
      {
         muMax = mu;
         s1x = s1x3D;
         s2x = s2x3D;
         s3x = s3x3D;
         p0x = p0x3D;
         s1y = s1y3D;
         s2y = s2y3D;
         s3y = s3y3D;
         p0y = p0y3D;
      }

      if (Math.abs(muMax) < epsilon)
         return null;

      double C1 = p0x * (s2y - s3y) + s2x * (s3y - p0y) + s3x * (p0y - s2y); // = area of the triangle (p0, s2, s3) projected onto the selected Cartesian plane.
      double C2 = s1x * (p0y - s3y) + p0x * (s3y - s1y) + s3x * (s1y - p0y); // = area of the triangle (s1, p0, s3) projected onto the selected Cartesian plane.
      double C3 = s1x * (s2y - p0y) + s2x * (p0y - s1y) + p0x * (s1y - s2y); // = area of the triangle (s1, s2, p0) projected onto the selected Cartesian plane.

      return new double[] {C1 / muMax, C2 / muMax, C3 / muMax};
   }

   private static List<EPAFace3D> fromDifferenceVertices(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, DifferenceVertex3D[] differenceVertices,
                                                         double epsilon)
   {
      List<EPAFace3D> epaPolytope = new ArrayList<>();

      if (differenceVertices.length == 4)
      {
         EPAVertex3D y0 = new EPAVertex3D(differenceVertices[0].getVertexOnShapeA(), differenceVertices[0].getVertexOnShapeB());
         EPAVertex3D y1 = new EPAVertex3D(differenceVertices[1].getVertexOnShapeA(), differenceVertices[1].getVertexOnShapeB());
         EPAVertex3D y2 = new EPAVertex3D(differenceVertices[2].getVertexOnShapeA(), differenceVertices[2].getVertexOnShapeB());
         EPAVertex3D y3 = new EPAVertex3D(differenceVertices[3].getVertexOnShapeA(), differenceVertices[3].getVertexOnShapeB());

         // Estimate the face's normal based on its vertices and knowing the expecting ordering based on the twin-edge: v1, v2, then v3.
         Vector3D n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(y0, y1, y1, y2);
         // As the vertices are clockwise ordered the cross-product of 2 successive edges should be negated to obtain the face's normal.
         n.negate();

         if (EuclidGeometryTools.isPoint3DAbovePlane3D(y3, y0, n))
         {
            EPAFace3D f0 = new EPAFace3D(y3, y0, y1, epsilon);
            EPAFace3D f1 = new EPAFace3D(y3, y1, y2, epsilon);
            EPAFace3D f2 = new EPAFace3D(y3, y2, y0, epsilon);
            EPAFace3D f3 = new EPAFace3D(y0, y2, y1, epsilon);

            f0.e1.setTwin(f3.e2); // e01 <-> e10
            f3.e0.setTwin(f2.e1); // e02 <-> e20
            f2.e2.setTwin(f0.e0); // e03 <-> e30

            f1.e1.setTwin(f3.e1); // e12 <-> e21
            f0.e2.setTwin(f1.e0); // e13 <-> e31
            f1.e2.setTwin(f2.e0); // e23 <-> e32

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else
         {
            EPAFace3D f0 = new EPAFace3D(y3, y1, y0, epsilon);
            EPAFace3D f1 = new EPAFace3D(y3, y2, y1, epsilon);
            EPAFace3D f2 = new EPAFace3D(y3, y0, y2, epsilon);
            EPAFace3D f3 = new EPAFace3D(y0, y1, y2, epsilon);

            f3.e0.setTwin(f0.e1); // e01 <-> e10
            f2.e1.setTwin(f3.e2); // e02 <-> e20
            f0.e2.setTwin(f2.e0); // e03 <-> e30

            f3.e1.setTwin(f1.e1); // e12 <-> e21
            f1.e2.setTwin(f0.e0); // e13 <-> e31
            f2.e2.setTwin(f1.e0); // e23 <-> e32

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
      }
      else if (differenceVertices.length == 3)
      {
         EPAVertex3D y0 = new EPAVertex3D(differenceVertices[0].getVertexOnShapeA(), differenceVertices[0].getVertexOnShapeB());
         EPAVertex3D y1 = new EPAVertex3D(differenceVertices[1].getVertexOnShapeA(), differenceVertices[1].getVertexOnShapeB());
         EPAVertex3D y2 = new EPAVertex3D(differenceVertices[2].getVertexOnShapeA(), differenceVertices[2].getVertexOnShapeB());

         // Estimate the face's normal based on its vertices and knowing the expecting ordering based on the twin-edge: v1, v2, then v3.
         Vector3D n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(y0, y1, y1, y2);
         // As the vertices are clockwise ordered the cross-product of 2 successive edges should be negated to obtain the face's normal.
         n.negate();

         Point3DReadOnly vertexA, vertexB;
         EPAVertex3D y3, y4;
         vertexA = shapeA.getSupportingVertex(n);
         n.negate();
         vertexB = shapeB.getSupportingVertex(n);
         y3 = new EPAVertex3D(vertexA, vertexB);
         vertexA = shapeA.getSupportingVertex(n);
         n.negate();
         vertexB = shapeB.getSupportingVertex(n);
         y4 = new EPAVertex3D(vertexA, vertexB);

         if (EuclidPolytopeTools.tetrahedronContainsOrigin(y0, y1, y2, y3))
         {
            EPAFace3D f0 = new EPAFace3D(y3, y0, y1, epsilon);
            EPAFace3D f1 = new EPAFace3D(y3, y1, y2, epsilon);
            EPAFace3D f2 = new EPAFace3D(y3, y2, y0, epsilon);
            EPAFace3D f3 = new EPAFace3D(y0, y2, y1, epsilon);

            f0.e1.setTwin(f3.e2); // e01 <-> e10
            f3.e0.setTwin(f2.e1); // e02 <-> e20
            f2.e2.setTwin(f0.e0); // e03 <-> e30

            f1.e1.setTwin(f3.e1); // e12 <-> e21
            f0.e2.setTwin(f1.e0); // e13 <-> e31
            f1.e2.setTwin(f2.e0); // e23 <-> e32

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else
         {
            EPAFace3D f0 = new EPAFace3D(y4, y1, y0, epsilon);
            EPAFace3D f1 = new EPAFace3D(y4, y2, y1, epsilon);
            EPAFace3D f2 = new EPAFace3D(y4, y0, y2, epsilon);
            EPAFace3D f3 = new EPAFace3D(y0, y1, y2, epsilon);

            f3.e0.setTwin(f0.e1); // e01 <-> e10
            f2.e1.setTwin(f3.e2); // e02 <-> e20
            f0.e2.setTwin(f2.e0); // e04 <-> e40

            f3.e1.setTwin(f1.e1); // e12 <-> e21
            f1.e2.setTwin(f0.e0); // e14 <-> e41
            f2.e2.setTwin(f1.e0); // e24 <-> e42

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         // TODO Add test for degenerate face
      }
      else if (differenceVertices.length == 2)
      {
         EPAVertex3D y0 = new EPAVertex3D(differenceVertices[0].getVertexOnShapeA(), differenceVertices[0].getVertexOnShapeB());
         EPAVertex3D y1 = new EPAVertex3D(differenceVertices[1].getVertexOnShapeA(), differenceVertices[1].getVertexOnShapeB());

         Vector3D d = new Vector3D();
         d.sub(y1, y0);

         Vector3DReadOnly axis = Axis.X;
         double coord = Math.abs(d.getX());
         double yAbs = Math.abs(d.getY());
         double zAbs = Math.abs(d.getZ());

         if (yAbs > coord)
         {
            coord = yAbs;
            axis = Axis.Y;
         }
         if (zAbs > coord)
         {
            axis = Axis.Z;
         }

         Vector3D v1 = new Vector3D();
         v1.cross(d, axis);
         RotationMatrix r = new RotationMatrix(new AxisAngle(d, 2.0 / 3.0 * Math.PI));
         Vector3D v2 = new Vector3D();
         Vector3D v3 = new Vector3D();
         r.transform(v1, v2);
         r.transform(v2, v3);

         Point3DReadOnly vertexA, vertexB;
         EPAVertex3D y2, y3, y4;
         vertexA = shapeA.getSupportingVertex(v1);
         v1.negate();
         vertexB = shapeB.getSupportingVertex(v1);
         y2 = new EPAVertex3D(vertexA, vertexB);
         vertexA = shapeA.getSupportingVertex(v2);
         v2.negate();
         vertexB = shapeB.getSupportingVertex(v2);
         y3 = new EPAVertex3D(vertexA, vertexB);
         vertexA = shapeA.getSupportingVertex(v3);
         v3.negate();
         vertexB = shapeB.getSupportingVertex(v3);
         y4 = new EPAVertex3D(vertexA, vertexB);

         if (EuclidPolytopeTools.tetrahedronContainsOrigin(y0, y2, y3, y4))
         {
            // Building the faces such that clockwise winding
            EPAFace3D f0 = new EPAFace3D(y0, y3, y2, epsilon);
            EPAFace3D f1 = new EPAFace3D(y0, y4, y3, epsilon);
            EPAFace3D f2 = new EPAFace3D(y0, y2, y4, epsilon);
            EPAFace3D f3 = new EPAFace3D(y2, y3, y4, epsilon);

            f2.e0.setTwin(f0.e2); // e02 <-> e20
            f0.e0.setTwin(f1.e2); // e03 <-> e30
            f1.e0.setTwin(f2.e2); // e04 <-> e40

            f2.e0.setTwin(f0.e1); // e23 <-> e32
            f2.e1.setTwin(f3.e2); // e24 <-> e42
            f3.e1.setTwin(f1.e1); // e34 <-> e43

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else
         {
            // Building the faces such that clockwise winding
            EPAFace3D f0 = new EPAFace3D(y1, y2, y3, epsilon);
            EPAFace3D f1 = new EPAFace3D(y1, y3, y4, epsilon);
            EPAFace3D f2 = new EPAFace3D(y1, y4, y2, epsilon);
            EPAFace3D f3 = new EPAFace3D(y2, y4, y3, epsilon);

            f0.e0.setTwin(f2.e2); // e12 <-> e21
            f1.e0.setTwin(f0.e2); // e13 <-> e31
            f2.e0.setTwin(f1.e2); // e14 <-> e41

            f0.e1.setTwin(f3.e2); // e23 <-> e32
            f3.e0.setTwin(f2.e1); // e24 <-> e42
            f1.e1.setTwin(f3.e1); // e34 <-> e43

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         // TODO Add test for degenerate face
      }
      else if (differenceVertices.length == 1)
      {
         // Supposedly this case only occurs when 2 shapes are only touching with 0-depth.
         return null;
      }

      return epaPolytope;
   }
}
