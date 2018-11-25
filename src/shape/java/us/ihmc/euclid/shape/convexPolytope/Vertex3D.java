package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * This class stores the location of a point which is the vertex of a polytope A list of polytope
 * edges originating from this vertex is also stored for ease of algorithm design Faces to which
 * this vertex belongs can be accessed by iterating through the list of edges
 *
 * @author Apoorv S
 *
 */
public class Vertex3D implements Vertex3DReadOnly, Point3DBasics
{
   private double x, y, z;
   /**
    * List of edges that start at this vertex. May be part of different faces
    */
   private final List<HalfEdge3D> associatedEdges = new ArrayList<>();

   public Vertex3D()
   {
      setToZero();
   }

   public Vertex3D(double x, double y, double z)
   {
      set(x, y, z);
   }

   public Vertex3D(Point3DReadOnly position)
   {
      set(position);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public List<HalfEdge3D> getAssociatedEdges()
   {
      return associatedEdges;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
   }

   /**
    * Method to remove a particular edge from the associated edge list
    *
    * @param edgeToAdd the associated edge that is to be removed. In case the edge specified is not on
    *           the list, no errors are thrown
    *
    */
   public void removeAssociatedEdge(HalfEdge3D edgeToAdd)
   {
      associatedEdges.remove(edgeToAdd);
   }

   /**
    * Remove all edges in the associated edge list
    */
   public void clearAssociatedEdgeList()
   {
      associatedEdges.clear();
   }

   /**
    * Add a {@code List<E>} of DCEL edges to the associated edge list. This invokes the
    * {@code addAssociatedEdge()} method and addition to the list follows the same set of rules
    *
    * @param edgeList a list of DCEL edges that must be added
    */
   public void addAssociatedEdges(List<? extends HalfEdge3D> edgeList)
   {
      for (int i = 0; i < edgeList.size(); i++)
      {
         addAssociatedEdge(edgeList.get(i));
      }
   }

   /**
    * Add a DCEL edge to the associated edge list. In case the edge is already on the associated edge
    * list no action is carried out. The check for whether an edge is already on the list is done by
    * comparing objects. Hence is possible to add a edge that geometrically equals an already existent
    * edge
    *
    * @param edge the DCEL edge to add to the associated edge list
    */
   public void addAssociatedEdge(HalfEdge3D edge)
   {
      if (!isAssociatedWithEdge(edge))
         associatedEdges.add(edge);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public int getNumberOfAssociatedEdges()
   {
      return getAssociatedEdges().size();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public boolean isAnyFaceMarked()
   {
      boolean isMarked = false;
      for (int i = 0; !isMarked && i < associatedEdges.size(); i++)
      {
         isMarked |= associatedEdges.get(i).getFace().isMarked();
      }
      return isMarked;
   }

   public void round(double epsilon)
   {
      setX(Math.round(getX() / epsilon) * epsilon);
      setY(Math.round(getY() / epsilon) * epsilon);
      setZ(Math.round(getZ() / epsilon) * epsilon);
   }

   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   @Override
   public void setZ(double z)
   {
      this.z = z;
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
      if (object instanceof Vertex3DReadOnly)
         return equals((Vertex3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      long hashCode = 1L;
      hashCode = EuclidHashCodeTools.addToHashCode(hashCode, x);
      hashCode = EuclidHashCodeTools.addToHashCode(hashCode, y);
      hashCode = EuclidHashCodeTools.addToHashCode(hashCode, z);
      return EuclidHashCodeTools.toIntHashCode(hashCode);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple3DString(this);
   }
}
