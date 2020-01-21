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

/**
 * Implementation of a convex polytope 3D with triangle faces.
 * <p>
 * The main use-case is to create a visualizable convex polytope given a {@code EPAVertex3D} or
 * {@code EPAFace3D} which can help debugging the {@code ExpandingPolytopeAlgorithm}.
 * </p>
 *
 * @author Sylvain Bertrand
 * @see ExpandingPolytopeAlgorithm
 */
public class EPAConvexPolytope3D implements ConvexPolytope3DReadOnly
{
   /** The list of the vertices composing this convex polytope. */
   private final List<EPAVertex3D> vertices = new ArrayList<>();
   /** The list of the half-edges composing this convex polytope. */
   private final List<EPAHalfEdge3D> halfEdges = new ArrayList<>();
   /** The list of the faces composing this convex polytope. */
   private final List<EPAFace3D> faces = new ArrayList<>();

   /** The tightest bounding box entirely containing this face. */
   private final Point3D centroid = new Point3D();
   /** The volume of this convex polytope. */
   private final double volume;

   /**
    * Assembles the faces, edges, and vertices linked to the given vertex.
    *
    * @param startVertex the vertex that belongs to the polytope to be assembled. Not modified,
    *                    reference saved.
    */
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

      halfEdges.stream().map(EPAHalfEdge3D::getOrigin).distinct().forEach(vertices::add);
      volume = EuclidPolytopeConstructionTools.computeConvexPolytope3DVolume(this, centroid);
   }

   /**
    * Assembles the faces, edges, and vertices linked to the given face.
    *
    * @param startFace the face that belongs to the polytope to be assembled. Not modified, reference
    *                  saved.
    */
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

      halfEdges.stream().map(EPAHalfEdge3D::getOrigin).distinct().forEach(vertices::add);
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

   /** {@inheritDoc} */
   @Override
   public BoundingBox3DReadOnly getBoundingBox()
   {
      BoundingBox3D boundingBox3D = new BoundingBox3D();
      boundingBox3D.setToNaN();
      faces.forEach(face -> boundingBox3D.combine(face.getBoundingBox()));
      return boundingBox3D;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DReadOnly getCentroid()
   {
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public double getVolume()
   {
      return volume;
   }

   /**
    * This implementation of {@code ConvexPolytope3DReadOnly} does not use a construction epsilon.
    *
    * @return 0
    */
   @Override
   public double getConstructionEpsilon()
   {
      return 0;
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends Vertex3DReadOnly> getVertices()
   {
      return vertices;
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends HalfEdge3DReadOnly> getHalfEdges()
   {
      return halfEdges;
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends Face3DReadOnly> getFaces()
   {
      return faces;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(ConvexPolytope3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ConvexPolytope3DReadOnly)
         return equals((ConvexPolytope3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this convex polytope
    * 3D.
    *
    * @return the hash code value for this convex polytope 3D.
    */
   @Override
   public int hashCode()
   {
      // Using ArrayList.hashCode() to combine the hash-codes of the vertices defining this face.
      return vertices.hashCode();
   }

   /**
    * Provides a {@code String} representation of this convex polytope 3D as follows:
    *
    * <pre>
    * EPA Convex polytope 3D: number of: [faces: 4, edges: 12, vertices: 4
    * Face list:
    *    centroid: ( 0.582, -0.023,  0.160 ), normal: ( 0.516, -0.673,  0.530 )
    *    centroid: ( 0.420,  0.176,  0.115 ), normal: (-0.038,  0.895, -0.444 )
    *    centroid: ( 0.264, -0.253, -0.276 ), normal: ( 0.506,  0.225, -0.833 )
    *    centroid: ( 0.198, -0.176, -0.115 ), normal: (-0.643, -0.374,  0.668 )
    * Edge list:
    *    [( 0.674,  0.482,  0.712 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.674,  0.482,  0.712 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.674,  0.482,  0.712 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.674,  0.482,  0.712 )]
    * Vertex list:
    *    ( 0.674,  0.482,  0.712 )
    *    ( 0.870,  0.251,  0.229 )
    *    ( 0.204, -0.803, -0.461 )
    *    (-0.283, -0.207, -0.595 )
    * </pre>
    *
    * @return the {@code String} representing this convex polytope 3D.
    */
   @Override
   public String toString()
   {
      return "EPA " + EuclidShapeIOTools.getConvexPolytope3DString(this);
   }
}