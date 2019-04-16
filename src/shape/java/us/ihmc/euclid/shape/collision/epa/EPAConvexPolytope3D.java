package us.ihmc.euclid.shape.collision.epa;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EPAConvexPolytope3D implements ConvexPolytope3DReadOnly
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
         halfEdges.add(face.getEdge0());
         halfEdges.add(face.getEdge1());
         halfEdges.add(face.getEdge2());
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
         halfEdges.add(face.getEdge0());
         halfEdges.add(face.getEdge1());
         halfEdges.add(face.getEdge2());
      }

      halfEdges.stream().map(EPAEdge3D::getOrigin).distinct().forEach(vertices::add);
      volume = EuclidPolytopeConstructionTools.computeConvexPolytope3DVolume(this, centroid);
   }

   private static void collectFacesRecursively(EPAFace3D face, Set<EPAFace3D> facesToPack)
   {
      if (face.isObsolete() || !facesToPack.add(face))
         return;

      collectFacesRecursively(face.getEdge0().getTwin().getFace(), facesToPack);
      collectFacesRecursively(face.getEdge1().getTwin().getFace(), facesToPack);
      collectFacesRecursively(face.getEdge2().getTwin().getFace(), facesToPack);
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