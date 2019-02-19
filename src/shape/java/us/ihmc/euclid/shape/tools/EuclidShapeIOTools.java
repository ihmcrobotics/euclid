package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.*;

import java.util.Collection;
import java.util.List;
import java.util.function.Function;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.collision.Shape3DCollisionTestResult;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidShapeIOTools
{
   public static String getBox3DString(Box3DReadOnly box3D)
   {
      return getBox3DString(DEFAULT_FORMAT, box3D);
   }

   public static String getBox3DString(String format, Box3DReadOnly box3D)
   {
      if (box3D == null)
         return "null";
      else
         return getBox3DString(format, box3D.getPosition(), box3D.getOrientation(), box3D.getSize());
   }

   public static String getBox3DString(Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return getBox3DString(DEFAULT_FORMAT, position, orientation, size);
   }

   /**
    * 
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    */
   public static String getBox3DString(String format, Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return "Box 3D: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + ", size: "
            + getTuple3DString(format, size) + "]";
   }

   public static String getCapsule3DString(Capsule3DReadOnly capsule3D)
   {
      return getCapsule3DString(DEFAULT_FORMAT, capsule3D);
   }

   public static String getCapsule3DString(String format, Capsule3DReadOnly capsule3D)
   {
      return getCapsule3DString(format, capsule3D.getPosition(), capsule3D.getAxis(), capsule3D.getLength(), capsule3D.getRadius());
   }

   public static String getCapsule3DString(Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return getCapsule3DString(DEFAULT_FORMAT, position, axis, length, radius);
   }

   /**
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    */
   public static String getCapsule3DString(String format, Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return "Capsule 3D: [position: " + getTuple3DString(format, position) + ", axis: " + getTuple3DString(format, axis) + ", length: "
            + String.format(format, length) + ", radius: " + String.format(format, radius) + "]";
   }

   public static String getCylinder3DString(Cylinder3DReadOnly cylinder3D)
   {
      return getCylinder3DString(DEFAULT_FORMAT, cylinder3D);
   }

   public static String getCylinder3DString(String format, Cylinder3DReadOnly cylinder3D)
   {
      return getCylinder3DString(format, cylinder3D.getPosition(), cylinder3D.getAxis(), cylinder3D.getLength(), cylinder3D.getRadius());
   }

   public static String getCylinder3DString(Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return getCylinder3DString(DEFAULT_FORMAT, position, axis, length, radius);
   }

   /**
    * <pre>
    * Cylinder 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    */
   public static String getCylinder3DString(String format, Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return "Cylinder 3D: [position: " + getTuple3DString(format, position) + ", axis: " + getTuple3DString(format, axis) + ", length: "
            + String.format(format, length) + ", radius: " + String.format(format, radius) + "]";
   }

   public static String getEllipsoid3DString(Ellipsoid3DReadOnly box3D)
   {
      return getEllipsoid3DString(DEFAULT_FORMAT, box3D);
   }

   public static String getEllipsoid3DString(String format, Ellipsoid3DReadOnly ellipsoid3D)
   {
      if (ellipsoid3D == null)
         return "null";
      else
         return getEllipsoid3DString(format, ellipsoid3D.getPosition(), ellipsoid3D.getOrientation(), ellipsoid3D.getRadii());
   }

   public static String getEllipsoid3DString(Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      return getEllipsoid3DString(DEFAULT_FORMAT, position, orientation, radii);
   }

   /**
    * 
    * <pre>
    * Ellipsoid 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), radii: ( 0.191,  0.719,  0.479 )]
    * </pre>
    */
   public static String getEllipsoid3DString(String format, Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      return "Ellipsoid 3D: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + ", radii: "
            + getTuple3DString(format, radii) + "]";
   }

   public static String getPointShape3DString(PointShape3DReadOnly pointShape3D)
   {
      return getPointShape3DString(DEFAULT_FORMAT, pointShape3D);
   }

   public static String getPointShape3DString(String format, PointShape3DReadOnly pointShape3D)
   {
      return getPointShape3DString(format, (Tuple3DReadOnly) pointShape3D);
   }

   public static String getPointShape3DString(String format, Tuple3DReadOnly position)
   {
      return "Point shape 3D: " + getTuple3DString(format, position);
   }

   public static String getRamp3DString(Ramp3DReadOnly ramp3D)
   {
      return getRamp3DString(DEFAULT_FORMAT, ramp3D);
   }

   public static String getRamp3DString(String format, Ramp3DReadOnly ramp3D)
   {
      if (ramp3D == null)
         return "null";
      else
         return getRamp3DString(format, ramp3D.getPosition(), ramp3D.getOrientation(), ramp3D.getSize());
   }

   public static String getRamp3DString(Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return getRamp3DString(DEFAULT_FORMAT, position, orientation, size);
   }

   /**
    * 
    * <pre>
    * Ramp 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    */
   public static String getRamp3DString(String format, Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return "Ramp 3D: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + ", size: "
            + getTuple3DString(format, size) + "]";
   }

   public static String getSphere3DString(Sphere3DReadOnly sphere3D)
   {
      return getSphere3DString(DEFAULT_FORMAT, sphere3D);
   }

   public static String getSphere3DString(String format, Sphere3DReadOnly sphere3D)
   {
      return getSphere3DString(format, sphere3D.getPosition(), sphere3D.getRadius());
   }

   public static String getSphere3DString(Tuple3DReadOnly position, double radius)
   {
      return getSphere3DString(DEFAULT_FORMAT, position, radius);
   }

   /**
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906]
    * </pre>
    */
   public static String getSphere3DString(String format, Tuple3DReadOnly position, double radius)
   {
      return "Sphere 3D: [position: " + getTuple3DString(format, position) + ", radius: " + String.format(format, radius) + "]";
   }

   public static String getTorus3DString(Torus3DReadOnly torus3D)
   {
      return getTorus3DString(DEFAULT_FORMAT, torus3D);
   }

   public static String getTorus3DString(String format, Torus3DReadOnly torus3D)
   {
      return getTorus3DString(format, torus3D.getPosition(), torus3D.getAxis(), torus3D.getRadius(), torus3D.getTubeRadius());
   }

   public static String getTorus3DString(Tuple3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      return getTorus3DString(DEFAULT_FORMAT, position, axis, radius, tubeRadius);
   }

   /**
    * <pre>
    * Torus 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), radius:  0.971, tube radius:  0.113]
    * </pre>
    */
   public static String getTorus3DString(String format, Tuple3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      return "Torus 3D: [position: " + getTuple3DString(format, position) + ", axis: " + getTuple3DString(format, axis) + ", radius: "
            + String.format(format, radius) + ", tube radius: " + String.format(format, tubeRadius) + "]";
   }

   public static String getShape3DPoseString(Shape3DPoseReadOnly shape3DPose)
   {
      return getShape3DPoseString(DEFAULT_FORMAT, shape3DPose.getShapeOrientation(), shape3DPose.getShapePosition());
   }

   public static String getShape3DPoseString(String format, Shape3DPoseReadOnly shape3DPose)
   {
      return getShape3DPoseString(format, shape3DPose.getShapeOrientation(), shape3DPose.getShapePosition());
   }

   public static String getShape3DPoseString(String format, RotationMatrixReadOnly orientation, Point3DReadOnly position)
   {
      return "Shape 3D pose: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + "]";
   }

   public static String getCollisionTestResultString(Shape3DCollisionTestResult collisionTestResult)
   {
      return getCollisionTestResultString(DEFAULT_FORMAT, collisionTestResult);
   }

   public static String getCollisionTestResultString(String format, Shape3DCollisionTestResult collisionTestResult)
   {
      if (collisionTestResult == null)
         return "null";

      String string = "Collision test result: ";
      if (collisionTestResult.areShapesColliding())
      {
         string += "colliding, depth: " + collisionTestResult.getDistance() + "\n";
      }
      else
      {
         string += "non-colliding, separating distance: " + collisionTestResult.getDistance() + "\n";
      }
      string += "Shape A: " + (collisionTestResult.getShapeA() == null ? "null" : collisionTestResult.getShapeA().getClass().getSimpleName());
      string += ", location: " + getTuple3DString(format, collisionTestResult.getPointOnA());
      string += ", normal: " + getTuple3DString(format, collisionTestResult.getNormalOnA()) + "\n";
      string += "Shape B: " + (collisionTestResult.getShapeB() == null ? "null" : collisionTestResult.getShapeB().getClass().getSimpleName());
      string += ", location: " + getTuple3DString(format, collisionTestResult.getPointOnB());
      string += ", normal: " + getTuple3DString(format, collisionTestResult.getNormalOnB());
      return string;
   }

   public static String getVertex3DString(Vertex3DReadOnly vertex3D)
   {
      return getVertex3DString(DEFAULT_FORMAT, vertex3D);
   }

   public static String getVertex3DString(String format, Vertex3DReadOnly vertex3D)
   {
      return getVertex3DString(format, vertex3D, vertex3D.getAssociatedEdges());
   }

   public static String getVertex3DString(String format, Tuple3DReadOnly vertex3DPosition, Collection<? extends HalfEdge3DReadOnly> vertexHalfEdges)
   {
      String string = "Vertex 3D: " + getTuple3DString(format, vertex3DPosition) + ", number of edges: " + vertexHalfEdges.size();
      string += getHalfEdge3DCollectionString(format, "\n\t", vertexHalfEdges);
      return string;
   }

   public static String getHalfEdge3DString(HalfEdge3DReadOnly halfEdge3D)
   {
      return getHalfEdge3DString(DEFAULT_FORMAT, halfEdge3D);
   }

   public static String getHalfEdge3DString(String format, HalfEdge3DReadOnly halfEdge3D)
   {
      if (halfEdge3D == null)
         return "null";
      return getHalfEdge3DString(format, halfEdge3D, halfEdge3D.getTwin(), halfEdge3D.getNext(), halfEdge3D.getPrevious(), halfEdge3D.getFace());
   }

   public static String getHalfEdge3DString(String format, LineSegment3DReadOnly edgeSupportingSegment, HalfEdge3DReadOnly twinEdge,
                                            HalfEdge3DReadOnly nextEdge, HalfEdge3DReadOnly previousEdge, Face3DReadOnly face)
   {
      String string = "Half-edge 3D: " + getLineSegment3DShortString(format, edgeSupportingSegment);
      string += "\n\tTwin    : " + getLineSegment3DShortString(format, twinEdge);
      string += "\n\tNext    : " + getLineSegment3DShortString(format, nextEdge);
      string += "\n\tPrevious: " + getLineSegment3DShortString(format, previousEdge);
      string += "\n\tFace: " + getFace3DShortString(format, face);
      return string;
   }

   public static String getFace3DString(Face3DReadOnly face3D)
   {
      return getFace3DString(DEFAULT_FORMAT, face3D);
   }

   public static String getFace3DString(String format, Face3DReadOnly face3D)
   {
      if (face3D == null)
         return "null";
      return getFace3DString(format, face3D.getCentroid(), face3D.getNormal(), face3D.getEdges());
   }

   public static String getFace3DString(String format, Point3DReadOnly centroid, Vector3DReadOnly normal, Collection<? extends HalfEdge3DReadOnly> faceEdges)
   {
      String string = "Face 3D: " + getFace3DShortString(format, centroid, normal) + ", number of edges: " + faceEdges.size()
            + getHalfEdge3DCollectionString(format, "\n\t", faceEdges);
      return string;
   }

   public static String getConvexPolytope3DString(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      return getConvexPolytope3DString(DEFAULT_FORMAT, convexPolytope3D);
   }

   public static String getConvexPolytope3DString(String format, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      if (convexPolytope3D == null)
         return "null";
      return getConvexPolytope3DString(format, convexPolytope3D.getVertices(), convexPolytope3D.getHalfEdges(), convexPolytope3D.getFaces());
   }

   public static String getConvexPolytope3DString(String format, Collection<? extends Vertex3DReadOnly> polytopeVertices,
                                                  Collection<? extends HalfEdge3DReadOnly> polytopeEdges, Collection<? extends Face3DReadOnly> polytopeFaces)
   {
      String string = "Convex polytope 3D: number of: [faces: " + polytopeFaces.size() + ", edges: " + polytopeEdges.size() + ", vertices: "
            + polytopeVertices.size();
      String linePrefix = "\n\t";
      string += "\nFace list: " + getFace3DCollectionString(format, linePrefix, polytopeFaces);
      string += "\nEdge list: " + getHalfEdge3DCollectionString(format, linePrefix, polytopeEdges);
      string += "\nVertex list: " + getVertex3DCollectionString(format, linePrefix, polytopeVertices);
      return string;
   }

   public static String getConvexPolytope3DStringForUnitTesting(ConvexPolytope3DReadOnly badPolytope, Point3DReadOnly troublingVertex)
   {
      return getConvexPolytope3DStringForUnitTesting(badPolytope.getVertices(), troublingVertex, badPolytope.getConstructionEpsilon());
   }

   public static String getConvexPolytope3DStringForUnitTesting(List<? extends Point3DReadOnly> verticesBeforeProblem, Point3DReadOnly troublingVertex,
                                                                double constructionEpsilon)
   {
      String stringFormat = EuclidCoreIOTools.getStringFormat(23, 20);

      String prefix = "vertices.add(new Point3D";
      String suffix = ");";
      String separator = suffix + "\n" + prefix;
      Function<Point3DReadOnly, String> elementToStringFunction = v -> EuclidCoreIOTools.getTuple3DString(stringFormat, v);

      String result = "";
      result += "List<Point3D> vertices = new ArrayList<>();\n";
      result += EuclidCoreIOTools.getCollectionString(prefix, suffix, separator, verticesBeforeProblem, elementToStringFunction) + "\n";
      result += "Point3D troublingVertex = new Point3D" + elementToStringFunction.apply(troublingVertex) + ";\n";
      result += "double constructionEpsilon = " + constructionEpsilon + ";\n";
      result += "ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices), constructionEpsilon);\n";
      result += "convexPolytope3D.addVertex(troublingVertex);";
      return result;
   }

   private static String getVertex3DCollectionString(String format, String linePrefix, Collection<? extends Vertex3DReadOnly> vertices)
   {
      return EuclidCoreIOTools.getCollectionString(linePrefix, null, linePrefix, vertices, vertex -> getTuple3DString(format, vertex));
   }

   private static String getHalfEdge3DCollectionString(String format, String linePrefix, Collection<? extends HalfEdge3DReadOnly> halfEdges)
   {
      return EuclidCoreIOTools.getCollectionString(linePrefix, null, linePrefix, halfEdges, halfEdge -> getLineSegment3DShortString(format, halfEdge));
   }

   private static String getFace3DCollectionString(String format, String linePrefix, Collection<? extends Face3DReadOnly> faces)
   {
      return EuclidCoreIOTools.getCollectionString(linePrefix, null, linePrefix, faces, face -> getFace3DShortString(format, face));
   }

   private static String getLineSegment3DShortString(String format, LineSegment3DReadOnly edgeSupportingSegment)
   {
      if (edgeSupportingSegment == null)
         return null;
      return "[" + getTuple3DString(format, edgeSupportingSegment.getFirstEndpoint()) + "; "
            + getTuple3DString(format, edgeSupportingSegment.getSecondEndpoint()) + "]";
   }

   private static String getFace3DShortString(String format, Face3DReadOnly face3D)
   {
      if (face3D == null)
         return "null";
      return getFace3DShortString(format, face3D.getCentroid(), face3D.getNormal());
   }

   private static String getFace3DShortString(String format, Point3DReadOnly faceCentroid, Vector3DReadOnly faceNormal)
   {
      return "centroid: " + getTuple3DString(format, faceCentroid) + ", normal: " + getTuple3DString(format, faceNormal);
   }
}
