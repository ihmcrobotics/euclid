package us.ihmc.euclid.shape.collision;

import static us.ihmc.euclid.shape.collision.GilbertJohnsonKeerthiCollisionDetector.*;

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
import us.ihmc.euclid.shape.collision.GilbertJohnsonKeerthiCollisionDetector.GJKVertex3D;
import us.ihmc.euclid.shape.collision.GilbertJohnsonKeerthiCollisionDetector.ProjectedTriangleSignedAreaCalculator;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
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
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class ExpandingPolytopeAlgorithm
{
   private static final boolean VERBOSE = false;

   private double epsilon = 1.0e-12;
   private int numberOfIterations = 0;
   private int maxIterations = 10000;
   private final GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private EPAFace3D lastResult = null;

   public ExpandingPolytopeAlgorithm()
   {
   }

   public EuclidShape3DCollisionResult evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   public void evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, EuclidShape3DCollisionResult resultToPack)
   {
      evaluateCollision((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
   }

   public EuclidShape3DCollisionResult evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   public boolean evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, EuclidShape3DCollisionResultBasics resultToPack)
   {
      boolean areShapesColliding = gjkCollisionDetector.evaluateCollision(shapeA, shapeB, resultToPack);
      if (areShapesColliding)
         evaluateCollision(shapeA, shapeB, gjkCollisionDetector.getSimplexVertices(), resultToPack);
      return areShapesColliding;
   }

   public void evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, GJKVertex3D[] simplex, EuclidShape3DCollisionResultBasics resultToPack)
   {
      PriorityQueue<EPAFace3D> queue = new PriorityQueue<>();
      double mu = Double.POSITIVE_INFINITY;

      List<EPAFace3D> initialPolytope = fromDifferenceVertices(shapeA, shapeB, simplex, epsilon);

      if (initialPolytope == null)
      {
         lastResult = null;
         if (VERBOSE)
            System.out.println("Initial polytope has triangle face that is affinely dependent.");
      }
      else
      {
         initialPolytope.stream().forEach(queue::add);
         Vector3D supportDirection = new Vector3D();
         numberOfIterations = 0;

         while (numberOfIterations < maxIterations)
         {
            if (queue.isEmpty())
            {
               if (VERBOSE)
                  System.out.println("Queue is empty, terminating.");
               break;
            }

            EPAFace3D entry = queue.poll();
            if (entry.isObsolete())
               continue;
            double currentNormSquared = entry.getNormSquared();

            numberOfIterations++;

            /*
             * In the original algorithm, the comparison does not add epsilon to mu, but this appears to be
             * needed to handle some edge-cases.
             */
            if (currentNormSquared > mu + epsilon)
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
                  System.out.println("New vertex equals to a vertex of entry, terminating.");
               break;
            }

            mu = Math.min(mu, square(TupleTools.dot(newVertex, supportDirection)) / currentNormSquared);

            if (mu <= square(1.0 + epsilon) * currentNormSquared)
            {
               if (VERBOSE)
                  System.out.println("Reached max accuracy, terminating.");
               break;
            }

            if (!entry.canObserverSeeFace(newVertex))
            {
               if (VERBOSE)
                  System.out.println("Malformed silhouette, terminating.");
               break;
            }

            entry.markObsolete();
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

               /*
                * Same as above, the original check with mu does not include epsilon, but it seems to be needed to
                * handle some edge-cases.
                */
               if (newEntry.isClosestPointInternal() && currentNormSquared <= newEntry.getNormSquared() && newEntry.getNormSquared() <= mu + epsilon)
               {
                  queue.add(newEntry);
               }
            }

            if (!areNewTrianglesFine)
               break;

            boolean terminate = false;

            for (int edgeIndex = 0; edgeIndex < newVertex.getNumberOfAssociatedEdges(); edgeIndex++)
            {
               EPAEdge3D edge = newVertex.getAssociatedEdge(edgeIndex);
               EPAEdge3D twin = edge.getDestination().getEdgeTo(newVertex);

               if (twin == null)
               {
                  if (VERBOSE)
                     System.out.println("Could not find twin of an edge, silhouette probably malformed, terminating.");
                  terminate = true;
                  break;
               }

               edge.setTwin(twin);
            }

            if (terminate)
               break;

            entry.destroy();
         }
      }

      if (initialPolytope == null)
      {
         resultToPack.setToNaN();
      }
      else
      {
         resultToPack.setShapesAreColliding(true);
         resultToPack.setDistance(-lastResult.getNorm());
         lastResult.computePointOnA(resultToPack.getPointOnA());
         lastResult.computePointOnB(resultToPack.getPointOnB());
         resultToPack.getNormalOnA().setToNaN();
         resultToPack.getNormalOnB().setToNaN();
      }

      if (VERBOSE)
         System.out.println("Number of iterations: " + numberOfIterations);
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }

   public EPAFace3D getClosestFace()
   {
      return lastResult;
   }

   private static double square(double value)
   {
      return value * value;
   }

   private static void silhouette(EPAEdge3D edge, Point3DReadOnly observer, List<EPAEdge3D> silhouetteToPack)
   {
      if (edge == null || edge.face.isObsolete())
         return;

      if (!edge.face.canObserverSeeFace(observer))
      {
         silhouetteToPack.add(edge);
      }
      else
      {
         edge.face.markObsolete();
         silhouette(edge.next.twin, observer, silhouetteToPack);
         silhouette(edge.prev.twin, observer, silhouetteToPack);
      }
   }

   public static class EPAConvexPolytope3D implements ConvexPolytope3DReadOnly
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
         if (face.isObsolete() || !facesToPack.add(face))
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

   public static class EPAFace3D implements Comparable<EPAFace3D>, Face3DReadOnly
   {
      private final EPAVertex3D v0, v1, v2;
      private final EPAEdge3D e0, e1, e2;
      private boolean obsolete = false;

      private final Point3DReadOnly closestPoint;
      private final double lambda0, lambda1, lambda2;
      private final boolean isTriangleAffinelyDependent;
      private final boolean isClosestPointInternal;
      private final double normSquared;
      private double norm = Double.NaN;
      private final Vector3D normal;

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

         normal = EuclidPolytopeTools.crossProductOfLineSegment3Ds(v1, v0, v1, v2);

         double[] lambdas = new double[3];
         SignedVolumeOutput output = signedVolumeFor2Simplex(v0, v1, v2, epsilon, lambdas);
         isTriangleAffinelyDependent = output == SignedVolumeOutput.AFFINELY_DEPENDENT;

         if (!isTriangleAffinelyDependent)
         {
            lambda0 = lambdas[0];
            lambda1 = lambdas[1];
            lambda2 = lambdas[2];

            isClosestPointInternal = output == SignedVolumeOutput.INSIDE;

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

      @Override
      public boolean canObserverSeeFace(Point3DReadOnly observer)
      {
         // The following test was suggested in the original algorithm.
         // However, it appears to not be robust to edge-cases where the new vertex would not be above 
         // the current entry.
         // if (TupleTools.dot(closestPoint, observer) >= normSquared)
         //    return true;
         return EuclidGeometryTools.isPoint3DAbovePlane3D(observer, v0, normal);
      }

      public double getNormSquared()
      {
         return normSquared;
      }

      public double getNorm()
      {
         if (Double.isNaN(norm))
            norm = Math.sqrt(getNormSquared());
         return norm;
      }

      public void destroy()
      {
         e0.destroy();
         e1.destroy();
         e2.destroy();
      }

      public void markObsolete()
      {
         obsolete = true;
      }

      public boolean isObsolete()
      {
         return obsolete;
      }

      public boolean isClosestPointInternal()
      {
         return isClosestPointInternal;
      }

      public void computePointOnA(Point3DBasics pointOnAToPack)
      {
         pointOnAToPack.setAndScale(lambda0, v0.getVertexOnShapeA());
         pointOnAToPack.scaleAdd(lambda1, v1.getVertexOnShapeA(), pointOnAToPack);
         pointOnAToPack.scaleAdd(lambda2, v2.getVertexOnShapeA(), pointOnAToPack);
      }

      public void computePointOnB(Point3DBasics pointOnBToPack)
      {
         pointOnBToPack.setAndScale(lambda0, v0.getVertexOnShapeB());
         pointOnBToPack.scaleAdd(lambda1, v1.getVertexOnShapeB(), pointOnBToPack);
         pointOnBToPack.scaleAdd(lambda2, v2.getVertexOnShapeB(), pointOnBToPack);
      }

      @Override
      public int compareTo(EPAFace3D other)
      {
         if (getNormSquared() == other.getNormSquared())
            return 0;
         if (getNormSquared() > other.getNormSquared())
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
         return normal;
      }

      @Override
      public String toString()
      {
         return EuclidShapeIOTools.getFace3DString(this);
      }
   }

   public static class EPAEdge3D implements HalfEdge3DReadOnly
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

   public static class EPAVertex3D implements Vertex3DReadOnly
   {
      private final double x, y, z;
      private final Point3DReadOnly vertexOnShapeA;
      private final Point3DReadOnly vertexOnShapeB;
      private final List<EPAEdge3D> associatedEdges = new ArrayList<>();

      public EPAVertex3D(GJKVertex3D gjkVertex3D)
      {
         vertexOnShapeA = gjkVertex3D.getVertexOnShapeA();
         vertexOnShapeB = gjkVertex3D.getVertexOnShapeB();
         x = gjkVertex3D.getX();
         y = gjkVertex3D.getY();
         z = gjkVertex3D.getZ();
      }

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
         if (!isEdgeAssociated(edge))
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

   public enum SignedVolumeOutput
   {
      INSIDE, OUTSIDE, AFFINELY_DEPENDENT
   };

   private static SignedVolumeOutput signedVolumeFor2Simplex(Point3DReadOnly s1, Point3DReadOnly s2, Point3DReadOnly s3, double epsilon, double[] lambdasToPack)
   {
      double s1x = s1.getX(), s1y = s1.getY(), s1z = s1.getZ();
      double s2x = s2.getX(), s2y = s2.getY(), s2z = s2.getZ();
      double s3x = s3.getX(), s3y = s3.getY(), s3z = s3.getZ();

      double p0x, p0y, p0z;
      { // Computing the projection p0 of the origin (0, 0, 0) onto the face as p0 = (s1.n) / (n.n) n
         double s2s1x3D = s1x - s2x;
         double s2s1y3D = s1y - s2y;
         double s2s1z3D = s1z - s2z;

         double s3s1x3D = s1x - s3x;
         double s3s1y3D = s1y - s3y;
         double s3s1z3D = s1z - s3z;

         // Computing the face's normal as n = (s2 - s1) x (s3 - s1)
         double nx = s2s1y3D * s3s1z3D - s2s1z3D * s3s1y3D;
         double ny = s2s1z3D * s3s1x3D - s2s1x3D * s3s1z3D;
         double nz = s2s1x3D * s3s1y3D - s2s1y3D * s3s1x3D;

         double distanceFromPlane = TupleTools.dot(nx, ny, nz, s1) / EuclidCoreTools.normSquared(nx, ny, nz);
         p0x = distanceFromPlane * nx;
         p0y = distanceFromPlane * ny;
         p0z = distanceFromPlane * nz;
      }

      double muMax = 0.0, muMaxAbs = 0.0; // mu = signed area of the triangle (s1, s2, s3)

      // Finding muMax and the coordinates that generate it.
      // As described in the paper, the 3D triangle is projected on each Cartesian plane.
      // The projected 2D triangle with the largest area is selected to pursue further computation.
      double mu, muAbs;
      ProjectedTriangleSignedAreaCalculator calculator = yzTriangleAreaCalculator;

      // Area of the triangle (s1, s2, s3) projected onto the YZ-plane.
      mu = calculator.compute(s1x, s1y, s1z, s2x, s2y, s2z, s3x, s3y, s3z);
      muAbs = Math.abs(mu);

      muMax = mu;
      muMaxAbs = muAbs;

      // Area of the triangle (s1, s2, s3) projected onto the XZ-plane.
      mu = zxTriangleAreaCalculator.compute(s1x, s1y, s1z, s2x, s2y, s2z, s3x, s3y, s3z);
      muAbs = Math.abs(mu);

      if (muAbs > muMaxAbs)
      {
         muMax = mu;
         muMaxAbs = muAbs;
         calculator = zxTriangleAreaCalculator;
      }

      // Area of the triangle (s1, s2, s3) projected onto the XY-plane.
      mu = xyTriangleAreaCalculator.compute(s1x, s1y, s1z, s2x, s2y, s2z, s3x, s3y, s3z);
      muAbs = Math.abs(mu);

      if (muAbs > muMaxAbs)
      {
         muMax = mu;
         muMaxAbs = muAbs;
         calculator = xyTriangleAreaCalculator;
      }

      if (Math.abs(muMax) < epsilon)
         return SignedVolumeOutput.AFFINELY_DEPENDENT;

      double C1 = calculator.compute(p0x, p0y, p0z, s2x, s2y, s2z, s3x, s3y, s3z); // = area of the triangle (p0, s2, s3) projected onto the selected Cartesian plane.
      double C2 = calculator.compute(s1x, s1y, s1z, p0x, p0y, p0z, s3x, s3y, s3z); // = area of the triangle (s1, p0, s3) projected onto the selected Cartesian plane.
      double C3 = calculator.compute(s1x, s1y, s1z, s2x, s2y, s2z, p0x, p0y, p0z); // = area of the triangle (s1, s2, p0) projected onto the selected Cartesian plane.

      if (compareSigns(muMax, C1) && compareSigns(muMax, C2) && compareSigns(muMax, C3))
      {
         lambdasToPack[0] = C1 / muMax;
         lambdasToPack[1] = C2 / muMax;
         lambdasToPack[2] = C3 / muMax;
         return SignedVolumeOutput.INSIDE;
      }
      else
      {
         double normSquared = Double.POSITIVE_INFINITY;
         boolean isAlmostInside = true;

         if (compareSigns(muMax, -C1))
         {
            if (Math.abs(C1) > epsilon)
               isAlmostInside = false;

            double[] candidateOutput = signedVolumeFor1Simplex(s2, s3);
            double lambda2 = candidateOutput[0];
            double lambda3 = candidateOutput[1];
            p0x = lambda2 * s2x + lambda3 * s3x;
            p0y = lambda2 * s2y + lambda3 * s3y;
            p0z = lambda2 * s2z + lambda3 * s3z;
            double candidateNormSquared = EuclidCoreTools.normSquared(p0x, p0y, p0z);
            lambdasToPack[0] = 0.0;
            lambdasToPack[1] = lambda2;
            lambdasToPack[2] = lambda3;
            normSquared = candidateNormSquared;
         }

         if (compareSigns(muMax, -C2))
         {
            if (Math.abs(C2) > epsilon)
               isAlmostInside = false;

            double[] candidateOutput = signedVolumeFor1Simplex(s1, s3);
            double lambda1 = candidateOutput[0];
            double lambda3 = candidateOutput[1];
            p0x = lambda1 * s1x + lambda3 * s3x;
            p0y = lambda1 * s1y + lambda3 * s3y;
            p0z = lambda1 * s1z + lambda3 * s3z;
            double candidateNormSquared = EuclidCoreTools.normSquared(p0x, p0y, p0z);
            if (candidateNormSquared < normSquared)
            {
               lambdasToPack[0] = lambda1;
               lambdasToPack[1] = 0.0;
               lambdasToPack[2] = lambda3;
               normSquared = candidateNormSquared;
            }
         }

         if (compareSigns(muMax, -C3))
         {
            if (Math.abs(C3) > epsilon)
               isAlmostInside = false;

            double[] candidateOutput = signedVolumeFor1Simplex(s1, s2);
            double lambda1 = candidateOutput[0];
            double lambda2 = candidateOutput[1];
            p0x = lambda1 * s1x + lambda2 * s2x;
            p0y = lambda1 * s1y + lambda2 * s2y;
            p0z = lambda1 * s1z + lambda2 * s2z;
            double candidateNormSquared = EuclidCoreTools.normSquared(p0x, p0y, p0z);
            if (candidateNormSquared < normSquared)
            {
               lambdasToPack[0] = lambda1;
               lambdasToPack[1] = lambda2;
               lambdasToPack[2] = 0.0;
               normSquared = candidateNormSquared;
            }
         }

         return isAlmostInside ? SignedVolumeOutput.INSIDE : SignedVolumeOutput.OUTSIDE;
      }
   }

   private static double[] signedVolumeFor1Simplex(Point3DReadOnly s1, Point3DReadOnly s2)
   {
      double s1x = s1.getX(), s1y = s1.getY(), s1z = s1.getZ();
      double s2x = s2.getX(), s2y = s2.getY(), s2z = s2.getZ();

      double p0x, p0y, p0z;
      { // Computing the projection p0 of the origin (0, 0, 0) onto the edge as p0 = s2 - (s2.t) / (t.t) t
        // Computing the edge's direction as t = (s2 - s1)
         double tx = s2x - s1x;
         double ty = s2y - s1y;
         double tz = s2z - s1z;

         double param = -TupleTools.dot(tx, ty, tz, s2) / EuclidCoreTools.normSquared(tx, ty, tz);
         p0x = param * tx + s2x;
         p0y = param * ty + s2y;
         p0z = param * tz + s2z;
      }

      double muMax = 0.0; // mu = length of the edge (s1, s2)
      double s1Max, s2Max, p0Max;

      { // Finding muMax and the coordinates that generate it.
        // Length of the edge (s1, s2) projected onto the X-axis.
         s1Max = s1x;
         s2Max = s2x;
         p0Max = p0x;
         muMax = s1x - s2x;

         // Length of the edge (s1, s2) projected onto the Y-axis.
         double mu = s1y - s2y;

         if (Math.abs(mu) > Math.abs(muMax))
         {
            s1Max = s1y;
            s2Max = s2y;
            p0Max = p0y;
            muMax = mu;
         }

         // Length of the edge (s1, s2) projected onto the Z-axis.
         mu = s1z - s2z;

         if (Math.abs(mu) > Math.abs(muMax))
         {
            s1Max = s1z;
            s2Max = s2z;
            p0Max = p0z;
            muMax = mu;
         }
      }

      double C1 = -(s2Max - p0Max);

      if (compareSigns(muMax, C1))
      {
         double C2 = s1Max - p0Max;

         if (compareSigns(muMax, C2))
         { // The projection in between the edge endpoints. Computing the barycentric coordinates.
            return new double[] {C1 / muMax, C2 / muMax};
         }
         else
         {
            return new double[] {0.0, 1.0};
         }
      }
      else
      {
         return new double[] {1.0, 0.0};
      }
   }

   private static List<EPAFace3D> fromDifferenceVertices(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, GJKVertex3D[] gjkVertices,
                                                         double epsilon)
   {
      List<EPAFace3D> epaPolytope = new ArrayList<>();

      if (gjkVertices == null)
      {
         return null;
      }
      else if (gjkVertices.length == 4)
      {
         EPAVertex3D y0 = new EPAVertex3D(gjkVertices[0]);
         EPAVertex3D y1 = new EPAVertex3D(gjkVertices[1]);
         EPAVertex3D y2 = new EPAVertex3D(gjkVertices[2]);
         EPAVertex3D y3 = new EPAVertex3D(gjkVertices[3]);

         // Estimate the face's normal based on its vertices and knowing the expecting ordering based on the twin-edge: v1, v2, then v3.
         Vector3D n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(y0, y1, y1, y2);
         // As the vertices are clockwise ordered the cross-product of 2 successive edges should be negated to obtain the face's normal.
         n.negate();

         if (EuclidGeometryTools.isPoint3DAbovePlane3D(y3, y0, n))
         {
            EPAFace3D f0 = new EPAFace3D(y3, y0, y1, epsilon);
            if (f0.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f1 = new EPAFace3D(y3, y1, y2, epsilon);
            if (f1.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f2 = new EPAFace3D(y3, y2, y0, epsilon);
            if (f2.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f3 = new EPAFace3D(y0, y2, y1, epsilon);
            if (f3.isTriangleAffinelyDependent)
               return null;

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
            if (f0.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f1 = new EPAFace3D(y3, y2, y1, epsilon);
            if (f1.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f2 = new EPAFace3D(y3, y0, y2, epsilon);
            if (f2.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f3 = new EPAFace3D(y0, y1, y2, epsilon);
            if (f3.isTriangleAffinelyDependent)
               return null;

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
      else if (gjkVertices.length == 3)
      {
         EPAVertex3D y0 = new EPAVertex3D(gjkVertices[0]);
         EPAVertex3D y1 = new EPAVertex3D(gjkVertices[1]);
         EPAVertex3D y2 = new EPAVertex3D(gjkVertices[2]);

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
            if (f0.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f1 = new EPAFace3D(y3, y1, y2, epsilon);
            if (f1.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f2 = new EPAFace3D(y3, y2, y0, epsilon);
            if (f2.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f3 = new EPAFace3D(y0, y2, y1, epsilon);
            if (f3.isTriangleAffinelyDependent)
               return null;

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
         else if (EuclidPolytopeTools.tetrahedronContainsOrigin(y0, y1, y2, y4))
         {
            EPAFace3D f0 = new EPAFace3D(y4, y1, y0, epsilon);
            if (f0.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f1 = new EPAFace3D(y4, y2, y1, epsilon);
            if (f1.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f2 = new EPAFace3D(y4, y0, y2, epsilon);
            if (f2.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f3 = new EPAFace3D(y0, y1, y2, epsilon);
            if (f3.isTriangleAffinelyDependent)
               return null;

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
         else
         {
            EPAFace3D f0 = new EPAFace3D(y4, y1, y0, epsilon);
            if (f0.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f1 = new EPAFace3D(y4, y2, y1, epsilon);
            if (f1.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f2 = new EPAFace3D(y4, y0, y2, epsilon);
            if (f2.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f3 = new EPAFace3D(y3, y0, y1, epsilon);
            if (f3.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f4 = new EPAFace3D(y3, y1, y2, epsilon);
            if (f4.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f5 = new EPAFace3D(y3, y2, y0, epsilon);
            if (f5.isTriangleAffinelyDependent)
               return null;

            f3.e0.setTwin(f5.e2); // e30 <-> e03
            f4.e0.setTwin(f3.e2); // e31 <-> e13
            f5.e0.setTwin(f4.e2); // e32 <-> e23

            f2.e0.setTwin(f0.e2); // e40 <-> e04
            f0.e0.setTwin(f1.e2); // e41 <-> e14
            f1.e0.setTwin(f2.e2); // e42 <-> e24

            f3.e1.setTwin(f0.e1); // e01 <-> e10
            f2.e1.setTwin(f5.e1); // e02 <-> e20
            f4.e1.setTwin(f1.e1); // e12 <-> e21

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
            epaPolytope.add(f4);
            epaPolytope.add(f5);
         }
      }
      else if (gjkVertices.length == 2)
      {
         EPAVertex3D y0 = new EPAVertex3D(gjkVertices[0]);
         EPAVertex3D y1 = new EPAVertex3D(gjkVertices[1]);

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
            EPAFace3D f0 = new EPAFace3D(y0, y2, y3, epsilon);
            if (f0.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f1 = new EPAFace3D(y0, y3, y4, epsilon);
            if (f1.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f2 = new EPAFace3D(y0, y4, y2, epsilon);
            if (f2.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f3 = new EPAFace3D(y2, y4, y3, epsilon);
            if (f3.isTriangleAffinelyDependent)
               return null;

            f0.e0.setTwin(f2.e2); // e02 <-> e20
            f1.e0.setTwin(f0.e2); // e03 <-> e30
            f2.e0.setTwin(f1.e2); // e04 <-> e40

            f0.e1.setTwin(f3.e2); // e23 <-> e32
            f3.e0.setTwin(f2.e1); // e24 <-> e42
            f1.e1.setTwin(f3.e1); // e34 <-> e43

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else if (EuclidPolytopeTools.tetrahedronContainsOrigin(y1, y2, y3, y4))
         {
            // Building the faces such that clockwise winding
            EPAFace3D f0 = new EPAFace3D(y1, y3, y2, epsilon);
            if (f0.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f1 = new EPAFace3D(y1, y4, y3, epsilon);
            if (f1.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f2 = new EPAFace3D(y1, y2, y4, epsilon);
            if (f2.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f3 = new EPAFace3D(y2, y3, y4, epsilon);
            if (f3.isTriangleAffinelyDependent)
               return null;

            f2.e0.setTwin(f0.e2); // e12 <-> e21
            f0.e0.setTwin(f1.e2); // e13 <-> e31
            f1.e0.setTwin(f2.e2); // e14 <-> e41

            f3.e0.setTwin(f0.e1); // e23 <-> e32
            f2.e1.setTwin(f3.e2); // e24 <-> e42
            f3.e1.setTwin(f1.e1); // e34 <-> e43

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else
         {
            EPAFace3D f0 = new EPAFace3D(y0, y2, y3, epsilon);
            if (f0.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f1 = new EPAFace3D(y0, y3, y4, epsilon);
            if (f1.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f2 = new EPAFace3D(y0, y4, y2, epsilon);
            if (f2.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f3 = new EPAFace3D(y1, y3, y2, epsilon);
            if (f3.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f4 = new EPAFace3D(y1, y4, y3, epsilon);
            if (f4.isTriangleAffinelyDependent)
               return null;
            EPAFace3D f5 = new EPAFace3D(y1, y2, y4, epsilon);
            if (f5.isTriangleAffinelyDependent)
               return null;

            f0.e0.setTwin(f2.e2); // e02 <-> e20
            f1.e0.setTwin(f0.e2); // e03 <-> e30
            f2.e0.setTwin(f1.e2); // e04 <-> e40

            f5.e0.setTwin(f3.e2); // e12 <-> e21
            f3.e0.setTwin(f4.e2); // e13 <-> e31
            f4.e0.setTwin(f5.e2); // e14 <-> e41

            f0.e1.setTwin(f3.e1); // e23 <-> e32
            f5.e1.setTwin(f2.e1); // e24 <-> e42
            f1.e1.setTwin(f4.e1); // e34 <-> e43

            f0.e1.setTwin(f3.e1); // e23 <-> e32
            f5.e1.setTwin(f2.e1); // e24 <-> e42
            f1.e1.setTwin(f4.e1); // e34 <-> e43

            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
            epaPolytope.add(f4);
            epaPolytope.add(f5);
         }
      }
      else if (gjkVertices.length == 1)
      {
         // Supposedly this case only occurs when 2 shapes are only touching with 0-depth.
         return null;
      }

      return epaPolytope;
   }
}
