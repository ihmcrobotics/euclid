package us.ihmc.euclid.shape.collision.epa;

import static us.ihmc.euclid.shape.collision.epa.EPATools.*;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.epa.EPATools.BarycentricCoordinatesOutput;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EPAFace3D implements Comparable<EPAFace3D>, Face3DReadOnly
{
   private final EPAVertex3D v0, v1, v2;
   private final EPAEdge3D e0;
   private final EPAEdge3D e1;
   private final EPAEdge3D e2;

   private final Point3DReadOnly closestPoint;
   private final double lambda0, lambda1, lambda2;
   private final boolean isTriangleAffinelyDependent;
   private final boolean isClosestPointInternal;
   private final double normSquared;

   private boolean obsolete = false;
   private double norm = Double.NaN;
   private final Vector3D normal;

   public static EPAFace3D fromVertexAndTwinEdge(EPAVertex3D vertex, EPAEdge3D twin, double epsilon)
   {
      EPAVertex3D v0 = twin.getDestination();
      EPAVertex3D v1 = twin.getOrigin();
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

      e0.setNext(e1);
      e1.setNext(e2);
      e2.setNext(e0);

      e0.setPrevious(e2);
      e1.setPrevious(e0);
      e2.setPrevious(e1);

      normal = EuclidPolytopeTools.crossProductOfLineSegment3Ds(v1, v0, v1, v2);

      double[] lambdas = new double[3];
      BarycentricCoordinatesOutput output = barycentricCoordinatesFrom2Simplex(v0, v1, v2, epsilon, lambdas);
      isTriangleAffinelyDependent = output == BarycentricCoordinatesOutput.AFFINELY_DEPENDENT;

      if (!isTriangleAffinelyDependent())
      {
         lambda0 = lambdas[0];
         lambda1 = lambdas[1];
         lambda2 = lambdas[2];

         isClosestPointInternal = output == BarycentricCoordinatesOutput.INSIDE;

         Point3D point = new Point3D();
         point.setAndScale(lambda0, v0);
         point.scaleAdd(lambda1, v1, point);
         point.scaleAdd(lambda2, v2, point);
         closestPoint = point;
         normSquared = getClosestPoint().distanceFromOriginSquared();
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

   public boolean isTriangleAffinelyDependent()
   {
      return isTriangleAffinelyDependent;
   }

   public EPAEdge3D getEdge0()
   {
      return e0;
   }

   public EPAEdge3D getEdge1()
   {
      return e1;
   }

   public EPAEdge3D getEdge2()
   {
      return e2;
   }

   public Point3DReadOnly getClosestPoint()
   {
      return closestPoint;
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