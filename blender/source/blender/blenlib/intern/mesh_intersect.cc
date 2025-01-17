/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/** \file
 * \ingroup bli
 */

/* The #blender::meshintersect API needs GMP. */
#ifdef WITH_GMP

#  include <algorithm>
#  include <fstream>
#  include <iostream>

#  include "BLI_allocator.hh"
#  include "BLI_array.hh"
#  include "BLI_assert.h"
#  include "BLI_delaunay_2d.h"
#  include "BLI_double3.hh"
#  include "BLI_float3.hh"
#  include "BLI_hash.hh"
#  include "BLI_kdopbvh.h"
#  include "BLI_map.hh"
#  include "BLI_math_boolean.hh"
#  include "BLI_math_mpq.hh"
#  include "BLI_mpq2.hh"
#  include "BLI_mpq3.hh"
#  include "BLI_span.hh"
#  include "BLI_task.h"
#  include "BLI_threads.h"
#  include "BLI_vector.hh"
#  include "BLI_vector_set.hh"

#  include "PIL_time.h"

#  include "BLI_mesh_intersect.hh"

//#  define PERFDEBUG

namespace blender::meshintersect {

#  ifdef PERFDEBUG
static void perfdata_init(void);
static void incperfcount(int countnum);
static void bumpperfcount(int countnum, int amt);
static void doperfmax(int maxnum, int val);
static void dump_perfdata(void);
#  endif

/** For debugging, can disable threading in intersect code with this static constant. */
static constexpr bool intersect_use_threading = true;

Vert::Vert(const mpq3 &mco, const double3 &dco, int id, int orig)
    : co_exact(mco), co(dco), id(id), orig(orig)
{
}

bool Vert::operator==(const Vert &other) const
{
  return this->co_exact == other.co_exact;
}

uint64_t Vert::hash() const
{
  return co_exact.hash();
}

std::ostream &operator<<(std::ostream &os, const Vert *v)
{
  os << "v" << v->id;
  if (v->orig != NO_INDEX) {
    os << "o" << v->orig;
  }
  os << v->co;
  return os;
}

bool Plane::operator==(const Plane &other) const
{
  return norm_exact == other.norm_exact && d_exact == other.d_exact;
}

void Plane::make_canonical()
{
  if (norm_exact[0] != 0) {
    mpq_class den = norm_exact[0];
    norm_exact = mpq3(1, norm_exact[1] / den, norm_exact[2] / den);
    d_exact = d_exact / den;
  }
  else if (norm_exact[1] != 0) {
    mpq_class den = norm_exact[1];
    norm_exact = mpq3(0, 1, norm_exact[2] / den);
    d_exact = d_exact / den;
  }
  else {
    if (norm_exact[2] != 0) {
      mpq_class den = norm_exact[2];
      norm_exact = mpq3(0, 0, 1);
      d_exact = d_exact / den;
    }
    else {
      /* A degenerate plane. */
      d_exact = 0;
    }
  }
  norm = double3(norm_exact[0].get_d(), norm_exact[1].get_d(), norm_exact[2].get_d());
  d = d_exact.get_d();
}

Plane::Plane(const mpq3 &norm_exact, const mpq_class &d_exact)
    : norm_exact(norm_exact), d_exact(d_exact)
{
  norm = double3(norm_exact[0].get_d(), norm_exact[1].get_d(), norm_exact[2].get_d());
  d = d_exact.get_d();
}

Plane::Plane(const double3 &norm, const double d) : norm(norm), d(d)
{
  norm_exact = mpq3(0, 0, 0); /* Marks as "exact not yet populated". */
}

/** This is wrong for degenerate planes, but we don't expect to call it on those. */
bool Plane::exact_populated() const
{
  return norm_exact[0] != 0 || norm_exact[1] != 0 || norm_exact[2] != 0;
}

uint64_t Plane::hash() const
{
  constexpr uint64_t h1 = 33;
  constexpr uint64_t h2 = 37;
  constexpr uint64_t h3 = 39;
  uint64_t hashx = hash_mpq_class(this->norm_exact.x);
  uint64_t hashy = hash_mpq_class(this->norm_exact.y);
  uint64_t hashz = hash_mpq_class(this->norm_exact.z);
  uint64_t hashd = hash_mpq_class(this->d_exact);
  uint64_t ans = hashx ^ (hashy * h1) ^ (hashz * h1 * h2) ^ (hashd * h1 * h2 * h3);
  return ans;
}

std::ostream &operator<<(std::ostream &os, const Plane *plane)
{
  os << "[" << plane->norm << ";" << plane->d << "]";
  return os;
}

Face::Face(
    Span<const Vert *> verts, int id, int orig, Span<int> edge_origs, Span<bool> is_intersect)
    : vert(verts), edge_orig(edge_origs), is_intersect(is_intersect), id(id), orig(orig)
{
}

Face::Face(Span<const Vert *> verts, int id, int orig) : vert(verts), id(id), orig(orig)
{
}

void Face::populate_plane(bool need_exact)
{
  if (plane != nullptr) {
    if (!need_exact || plane->exact_populated()) {
      return;
    }
  }
  if (need_exact) {
    mpq3 normal_exact;
    if (vert.size() > 3) {
      Array<mpq3> co(vert.size());
      for (int i : index_range()) {
        co[i] = vert[i]->co_exact;
      }
      normal_exact = mpq3::cross_poly(co);
    }
    else {
      mpq3 tr02 = vert[0]->co_exact - vert[2]->co_exact;
      mpq3 tr12 = vert[1]->co_exact - vert[2]->co_exact;
      normal_exact = mpq3::cross(tr02, tr12);
    }
    mpq_class d_exact = -mpq3::dot(normal_exact, vert[0]->co_exact);
    plane = new Plane(normal_exact, d_exact);
  }
  else {
    double3 normal;
    if (vert.size() > 3) {
      Array<double3> co(vert.size());
      for (int i : index_range()) {
        co[i] = vert[i]->co;
      }
      normal = double3::cross_poly(co);
    }
    else {
      double3 tr02 = vert[0]->co - vert[2]->co;
      double3 tr12 = vert[1]->co - vert[2]->co;
      normal = double3::cross_high_precision(tr02, tr12);
    }
    double d = -double3::dot(normal, vert[0]->co);
    plane = new Plane(normal, d);
  }
}

Face::~Face()
{
  delete plane;
}

bool Face::operator==(const Face &other) const
{
  if (this->size() != other.size()) {
    return false;
  }
  for (FacePos i : index_range()) {
    /* Can test pointer equality since we will have
     * unique vert pointers for unique co_equal's. */
    if (this->vert[i] != other.vert[i]) {
      return false;
    }
  }
  return true;
}

bool Face::cyclic_equal(const Face &other) const
{
  if (this->size() != other.size()) {
    return false;
  }
  int flen = this->size();
  for (FacePos start : index_range()) {
    for (FacePos start_other : index_range()) {
      bool ok = true;
      for (int i = 0; ok && i < flen; ++i) {
        FacePos p = (start + i) % flen;
        FacePos p_other = (start_other + i) % flen;
        if (this->vert[p] != other.vert[p_other]) {
          ok = false;
        }
      }
      if (ok) {
        return true;
      }
    }
  }
  return false;
}

std::ostream &operator<<(std::ostream &os, const Face *f)
{
  os << "f" << f->id << "o" << f->orig << "[";
  for (const Vert *v : *f) {
    os << "v" << v->id;
    if (v->orig != NO_INDEX) {
      os << "o" << v->orig;
    }
    if (v != f->vert[f->size() - 1]) {
      os << " ";
    }
  }
  os << "]";
  if (f->orig != NO_INDEX) {
    os << "o" << f->orig;
  }
  os << " e_orig[";
  for (int i : f->index_range()) {
    os << f->edge_orig[i];
    if (f->is_intersect[i]) {
      os << "#";
    }
    if (i != f->size() - 1) {
      os << " ";
    }
  }
  os << "]";
  return os;
}

/**
 * Un-comment the following to try using a spin-lock instead of
 * a mutex in the arena allocation routines.
 * Initial tests showed that it doesn't seem to help very much,
 * if at all, to use a spin-lock.
 */
// #define USE_SPINLOCK

/**
 * #IMeshArena is the owner of the Vert and Face resources used
 * during a run of one of the mesh-intersect main functions.
 * It also keeps has a hash table of all Verts created so that it can
 * ensure that only one instance of a Vert with a given co_exact will
 * exist. I.e., it de-duplicates the vertices.
 */
class IMeshArena::IMeshArenaImpl : NonCopyable, NonMovable {

  /**
   * Don't use Vert itself as key since resizing may move
   * pointers to the Vert around, and we need to have those pointers
   * stay the same throughout the lifetime of the #IMeshArena.
   */
  struct VSetKey {
    Vert *vert;

    VSetKey(Vert *p) : vert(p)
    {
    }

    uint32_t hash() const
    {
      return vert->hash();
    }

    bool operator==(const VSetKey &other) const
    {
      return *this->vert == *other.vert;
    }
  };

  VectorSet<VSetKey> vset_; /* TODO: replace with Set */

  /**
   * Ownership of the Vert memory is here, so destroying this reclaims that memory.
   *
   * TODO: replace these with pooled allocation, and just destroy the pools at the end.
   */
  Vector<std::unique_ptr<Vert>> allocated_verts_;
  Vector<std::unique_ptr<Face>> allocated_faces_;

  /* Use these to allocate ids when Verts and Faces are allocated. */
  int next_vert_id_ = 0;
  int next_face_id_ = 0;

  /* Need a lock when multi-threading to protect allocation of new elements. */
#  ifdef USE_SPINLOCK
  SpinLock lock_;
#  else
  ThreadMutex *mutex_;
#  endif

 public:
  IMeshArenaImpl()
  {
    if (intersect_use_threading) {
#  ifdef USE_SPINLOCK
      BLI_spin_init(&lock_);
#  else
      mutex_ = BLI_mutex_alloc();
#  endif
    }
  }
  ~IMeshArenaImpl()
  {
    if (intersect_use_threading) {
#  ifdef USE_SPINLOCK
      BLI_spin_end(&lock_);
#  else
      BLI_mutex_free(mutex_);
#  endif
    }
  }

  void reserve(int vert_num_hint, int face_num_hint)
  {
    vset_.reserve(vert_num_hint);
    allocated_verts_.reserve(vert_num_hint);
    allocated_faces_.reserve(face_num_hint);
  }

  int tot_allocated_verts() const
  {
    return allocated_verts_.size();
  }

  int tot_allocated_faces() const
  {
    return allocated_faces_.size();
  }

  const Vert *add_or_find_vert(const mpq3 &co, int orig)
  {
    double3 dco(co[0].get_d(), co[1].get_d(), co[2].get_d());
    return add_or_find_vert(co, dco, orig);
  }

  const Vert *add_or_find_vert(const double3 &co, int orig)
  {
    mpq3 mco(co[0], co[1], co[2]);
    return add_or_find_vert(mco, co, orig);
  }

  Face *add_face(Span<const Vert *> verts, int orig, Span<int> edge_origs, Span<bool> is_intersect)
  {
    Face *f = new Face(verts, next_face_id_++, orig, edge_origs, is_intersect);
    if (intersect_use_threading) {
#  ifdef USE_SPINLOCK
      BLI_spin_lock(&lock_);
#  else
      BLI_mutex_lock(mutex_);
#  endif
    }
    allocated_faces_.append(std::unique_ptr<Face>(f));
    if (intersect_use_threading) {
#  ifdef USE_SPINLOCK
      BLI_spin_unlock(&lock_);
#  else
      BLI_mutex_unlock(mutex_);
#  endif
    }
    return f;
  }

  Face *add_face(Span<const Vert *> verts, int orig, Span<int> edge_origs)
  {
    Array<bool> is_intersect(verts.size(), false);
    return add_face(verts, orig, edge_origs, is_intersect);
  }

  Face *add_face(Span<const Vert *> verts, int orig)
  {
    Array<int> edge_origs(verts.size(), NO_INDEX);
    Array<bool> is_intersect(verts.size(), false);
    return add_face(verts, orig, edge_origs, is_intersect);
  }

  const Vert *find_vert(const mpq3 &co)
  {
    const Vert *ans;
    Vert vtry(co, double3(), NO_INDEX, NO_INDEX);
    VSetKey vskey(&vtry);
    if (intersect_use_threading) {
#  ifdef USE_SPINLOCK
      BLI_spin_lock(&lock_);
#  else
      BLI_mutex_lock(mutex_);
#  endif
    }
    int i = vset_.index_of_try(vskey);
    if (i == -1) {
      ans = nullptr;
    }
    else {
      ans = vset_[i].vert;
    }
    if (intersect_use_threading) {
#  ifdef USE_SPINLOCK
      BLI_spin_unlock(&lock_);
#  else
      BLI_mutex_unlock(mutex_);
#  endif
    }
    return ans;
  }

  /**
   * This is slow. Only used for unit tests right now.
   * Since it is only used for that purpose, access is not lock-protected.
   * The argument vs can be a cyclic shift of the actual stored Face.
   */
  const Face *find_face(Span<const Vert *> vs)
  {
    Array<int> eorig(vs.size(), NO_INDEX);
    Array<bool> is_intersect(vs.size(), false);
    Face ftry(vs, NO_INDEX, NO_INDEX, eorig, is_intersect);
    for (const int i : allocated_faces_.index_range()) {
      if (ftry.cyclic_equal(*allocated_faces_[i])) {
        return allocated_faces_[i].get();
      }
    }
    return nullptr;
  }

 private:
  const Vert *add_or_find_vert(const mpq3 &mco, const double3 &dco, int orig)
  {
    /* Don't allocate Vert yet, in case it is already there. */
    Vert vtry(mco, dco, NO_INDEX, NO_INDEX);
    const Vert *ans;
    VSetKey vskey(&vtry);
    if (intersect_use_threading) {
#  ifdef USE_SPINLOCK
      BLI_spin_lock(&lock_);
#  else
      BLI_mutex_lock(mutex_);
#  endif
    }
    int i = vset_.index_of_try(vskey);
    if (i == -1) {
      vskey.vert = new Vert(mco, dco, next_vert_id_++, orig);
      vset_.add_new(vskey);
      allocated_verts_.append(std::unique_ptr<Vert>(vskey.vert));
      ans = vskey.vert;
    }
    else {
      /* It was a duplicate, so return the existing one.
       * Note that the returned Vert may have a different orig.
       * This is the intended semantics: if the Vert already
       * exists then we are merging verts and using the first-seen
       * one as the canonical one. */
      ans = vset_[i].vert;
    }
    if (intersect_use_threading) {
#  ifdef USE_SPINLOCK
      BLI_spin_unlock(&lock_);
#  else
      BLI_mutex_unlock(mutex_);
#  endif
    }
    return ans;
  };
};

IMeshArena::IMeshArena()
{
  pimpl_ = std::unique_ptr<IMeshArenaImpl>(new IMeshArenaImpl());
}

IMeshArena::~IMeshArena()
{
}

void IMeshArena::reserve(int vert_num_hint, int face_num_hint)
{
  pimpl_->reserve(vert_num_hint, face_num_hint);
}

int IMeshArena::tot_allocated_verts() const
{
  return pimpl_->tot_allocated_verts();
}

int IMeshArena::tot_allocated_faces() const
{
  return pimpl_->tot_allocated_faces();
}

const Vert *IMeshArena::add_or_find_vert(const mpq3 &co, int orig)
{
  return pimpl_->add_or_find_vert(co, orig);
}

Face *IMeshArena::add_face(Span<const Vert *> verts,
                           int orig,
                           Span<int> edge_origs,
                           Span<bool> is_intersect)
{
  return pimpl_->add_face(verts, orig, edge_origs, is_intersect);
}

Face *IMeshArena::add_face(Span<const Vert *> verts, int orig, Span<int> edge_origs)
{
  return pimpl_->add_face(verts, orig, edge_origs);
}

Face *IMeshArena::add_face(Span<const Vert *> verts, int orig)
{
  return pimpl_->add_face(verts, orig);
}

const Vert *IMeshArena::add_or_find_vert(const double3 &co, int orig)
{
  return pimpl_->add_or_find_vert(co, orig);
}

const Vert *IMeshArena::find_vert(const mpq3 &co) const
{
  return pimpl_->find_vert(co);
}

const Face *IMeshArena::find_face(Span<const Vert *> verts) const
{
  return pimpl_->find_face(verts);
}

void IMesh::set_faces(Span<Face *> faces)
{
  face_ = faces;
}

int IMesh::lookup_vert(const Vert *v) const
{
  BLI_assert(vert_populated_);
  return vert_to_index_.lookup_default(v, NO_INDEX);
}

void IMesh::populate_vert()
{
  /* This is likely an overestimate, since verts are shared between
   * faces. It is ok if estimate is over or even under. */
  constexpr int ESTIMATE_VERTS_PER_FACE = 4;
  int estimate_num_verts = ESTIMATE_VERTS_PER_FACE * face_.size();
  populate_vert(estimate_num_verts);
}

void IMesh::populate_vert(int max_verts)
{
  if (vert_populated_) {
    return;
  }
  vert_to_index_.reserve(max_verts);
  int next_allocate_index = 0;
  for (const Face *f : face_) {
    for (const Vert *v : *f) {
      if (v->id == 1) {
      }
      int index = vert_to_index_.lookup_default(v, NO_INDEX);
      if (index == NO_INDEX) {
        BLI_assert(next_allocate_index < UINT_MAX - 2);
        vert_to_index_.add(v, next_allocate_index++);
      }
    }
  }
  int tot_v = next_allocate_index;
  vert_ = Array<const Vert *>(tot_v);
  for (auto item : vert_to_index_.items()) {
    int index = item.value;
    BLI_assert(index < tot_v);
    vert_[index] = item.key;
  }
  /* Easier debugging (at least when there are no merged input verts)
   * if output vert order is same as input, with new verts at the end.
   * TODO: when all debugged, set fix_order = false. */
  const bool fix_order = true;
  if (fix_order) {
    std::sort(vert_.begin(), vert_.end(), [](const Vert *a, const Vert *b) {
      if (a->orig != NO_INDEX && b->orig != NO_INDEX) {
        return a->orig < b->orig;
      }
      if (a->orig != NO_INDEX) {
        return true;
      }
      if (b->orig != NO_INDEX) {
        return false;
      }
      return a->id < b->id;
    });
    for (int i : vert_.index_range()) {
      const Vert *v = vert_[i];
      vert_to_index_.add_overwrite(v, i);
    }
  }
  vert_populated_ = true;
}

void IMesh::erase_face_positions(int f_index, Span<bool> face_pos_erase, IMeshArena *arena)
{
  const Face *cur_f = this->face(f_index);
  int cur_len = cur_f->size();
  int num_to_erase = 0;
  for (int i : cur_f->index_range()) {
    if (face_pos_erase[i]) {
      ++num_to_erase;
    }
  }
  if (num_to_erase == 0) {
    return;
  }
  int new_len = cur_len - num_to_erase;
  if (new_len < 3) {
    /* Invalid erase. Don't do anything. */
    return;
  }
  Array<const Vert *> new_vert(new_len);
  Array<int> new_edge_orig(new_len);
  Array<bool> new_is_intersect(new_len);
  int new_index = 0;
  for (int i : cur_f->index_range()) {
    if (!face_pos_erase[i]) {
      new_vert[new_index] = (*cur_f)[i];
      new_edge_orig[new_index] = cur_f->edge_orig[i];
      new_is_intersect[new_index] = cur_f->is_intersect[i];
      ++new_index;
    }
  }
  BLI_assert(new_index == new_len);
  this->face_[f_index] = arena->add_face(new_vert, cur_f->orig, new_edge_orig, new_is_intersect);
}

std::ostream &operator<<(std::ostream &os, const IMesh &mesh)
{
  if (mesh.has_verts()) {
    os << "Verts:\n";
    int i = 0;
    for (const Vert *v : mesh.vertices()) {
      os << i << ": " << v << "\n";
      ++i;
    }
  }
  os << "\nFaces:\n";
  int i = 0;
  for (const Face *f : mesh.faces()) {
    os << i << ": " << f << "\n";
    if (f->plane != nullptr) {
      os << "    plane=" << f->plane << " eorig=[";
      for (Face::FacePos p = 0; p < f->size(); ++p) {
        os << f->edge_orig[p] << " ";
      }
      os << "]\n";
    }
    ++i;
  }
  return os;
}

struct BoundingBox {
  float3 min{FLT_MAX, FLT_MAX, FLT_MAX};
  float3 max{-FLT_MAX, -FLT_MAX, -FLT_MAX};

  BoundingBox() = default;
  BoundingBox(const float3 &min, const float3 &max) : min(min), max(max)
  {
  }
  BoundingBox(const BoundingBox &other) : min(other.min), max(other.max)
  {
  }
  BoundingBox(BoundingBox &&other) noexcept : min(std::move(other.min)), max(std::move(other.max))
  {
  }
  ~BoundingBox() = default;
  BoundingBox operator=(const BoundingBox &other)
  {
    if (this != &other) {
      min = other.min;
      max = other.max;
    }
    return *this;
  }
  BoundingBox operator=(BoundingBox &&other) noexcept
  {
    min = std::move(other.min);
    max = std::move(other.max);
    return *this;
  }

  void combine(const float3 &p)
  {
    min.x = min_ff(min.x, p.x);
    min.y = min_ff(min.y, p.y);
    min.z = min_ff(min.z, p.z);
    max.x = max_ff(max.x, p.x);
    max.y = max_ff(max.y, p.y);
    max.z = max_ff(max.z, p.z);
  }

  void combine(const double3 &p)
  {
    min.x = min_ff(min.x, static_cast<float>(p.x));
    min.y = min_ff(min.y, static_cast<float>(p.y));
    min.z = min_ff(min.z, static_cast<float>(p.z));
    max.x = max_ff(max.x, static_cast<float>(p.x));
    max.y = max_ff(max.y, static_cast<float>(p.y));
    max.z = max_ff(max.z, static_cast<float>(p.z));
  }

  void combine(const BoundingBox &bb)
  {
    min.x = min_ff(min.x, bb.min.x);
    min.y = min_ff(min.y, bb.min.y);
    min.z = min_ff(min.z, bb.min.z);
    max.x = max_ff(max.x, bb.max.x);
    max.y = max_ff(max.y, bb.max.y);
    max.z = max_ff(max.z, bb.max.z);
  }

  void expand(float pad)
  {
    min.x -= pad;
    min.y -= pad;
    min.z -= pad;
    max.x += pad;
    max.y += pad;
    max.z += pad;
  }
};

/**
 * Assume bounding boxes have been expanded by a sufficient epsilon on all sides
 * so that the comparisons against the bb bounds are sufficient to guarantee that
 * if an overlap or even touching could happen, this will return true.
 */
static bool bbs_might_intersect(const BoundingBox &bb_a, const BoundingBox &bb_b)
{
  return isect_aabb_aabb_v3(bb_a.min, bb_a.max, bb_b.min, bb_b.max);
}

/**
 * Data and functions to calculate bounding boxes and pad them, in parallel.
 * The bounding box calculation has the additional task of calculating the maximum
 * absolute value of any coordinate in the mesh, which will be used to calculate
 * the pad value.
 */
struct BBChunkData {
  double max_abs_val = 0.0;
};

struct BBCalcData {
  const IMesh &im;
  Array<BoundingBox> *face_bounding_box;

  BBCalcData(const IMesh &im, Array<BoundingBox> *fbb) : im(im), face_bounding_box(fbb){};
};

static void calc_face_bb_range_func(void *__restrict userdata,
                                    const int iter,
                                    const TaskParallelTLS *__restrict tls)
{
  BBCalcData *bbdata = static_cast<BBCalcData *>(userdata);
  double max_abs = 0.0;
  const Face &face = *bbdata->im.face(iter);
  BoundingBox &bb = (*bbdata->face_bounding_box)[iter];
  for (const Vert *v : face) {
    bb.combine(v->co);
    for (int i = 0; i < 3; ++i) {
      max_abs = max_dd(max_abs, fabs(v->co[i]));
    }
  }
  BBChunkData *chunk = static_cast<BBChunkData *>(tls->userdata_chunk);
  chunk->max_abs_val = max_dd(max_abs, chunk->max_abs_val);
}

struct BBPadData {
  Array<BoundingBox> *face_bounding_box;
  double pad;

  BBPadData(Array<BoundingBox> *fbb, double pad) : face_bounding_box(fbb), pad(pad){};
};

static void pad_face_bb_range_func(void *__restrict userdata,
                                   const int iter,
                                   const TaskParallelTLS *__restrict UNUSED(tls))
{
  BBPadData *pad_data = static_cast<BBPadData *>(userdata);
  (*pad_data->face_bounding_box)[iter].expand(pad_data->pad);
}

static void calc_face_bb_reduce(const void *__restrict UNUSED(userdata),
                                void *__restrict chunk_join,
                                void *__restrict chunk)
{
  BBChunkData *bbchunk_join = static_cast<BBChunkData *>(chunk_join);
  BBChunkData *bbchunk = static_cast<BBChunkData *>(chunk);
  bbchunk_join->max_abs_val = max_dd(bbchunk_join->max_abs_val, bbchunk->max_abs_val);
}

/**
 * We will expand the bounding boxes by an epsilon on all sides so that
 * the "less than" tests in isect_aabb_aabb_v3 are sufficient to detect
 * touching or overlap.
 */
static Array<BoundingBox> calc_face_bounding_boxes(const IMesh &m)
{
  int n = m.face_size();
  Array<BoundingBox> ans(n);
  TaskParallelSettings settings;
  BBCalcData data(m, &ans);
  BBChunkData chunk_data;
  BLI_parallel_range_settings_defaults(&settings);
  settings.userdata_chunk = &chunk_data;
  settings.userdata_chunk_size = sizeof(chunk_data);
  settings.func_reduce = calc_face_bb_reduce;
  settings.min_iter_per_thread = 1000;
  settings.use_threading = intersect_use_threading;
  BLI_task_parallel_range(0, n, &data, calc_face_bb_range_func, &settings);
  double max_abs_val = chunk_data.max_abs_val;
  constexpr float pad_factor = 10.0f;
  float pad = max_abs_val == 0.0f ? FLT_EPSILON : 2 * FLT_EPSILON * max_abs_val;
  pad *= pad_factor; /* For extra safety. */
  TaskParallelSettings pad_settings;
  BLI_parallel_range_settings_defaults(&pad_settings);
  settings.min_iter_per_thread = 1000;
  settings.use_threading = intersect_use_threading;
  BBPadData pad_data(&ans, pad);
  BLI_task_parallel_range(0, n, &pad_data, pad_face_bb_range_func, &pad_settings);
  return ans;
}

/**
 * A cluster of co-planar triangles, by index.
 * A pair of triangles T0 and T1 is said to "non-trivially co-planar-intersect"
 * if they are co-planar, intersect, and their intersection is not just existing
 * elements (verts, edges) of both triangles.
 * A co-planar cluster is said to be "nontrivial" if it has more than one triangle
 * and every triangle in it non-trivially co-planar-intersects with at least one other
 * triangle in the cluster.
 */
class CoplanarCluster {
  Vector<int> tris_;
  BoundingBox bb_;

 public:
  CoplanarCluster() = default;
  CoplanarCluster(int t, const BoundingBox &bb)
  {
    this->add_tri(t, bb);
  }
  CoplanarCluster(const CoplanarCluster &other) : tris_(other.tris_), bb_(other.bb_)
  {
  }
  CoplanarCluster(CoplanarCluster &&other) noexcept
      : tris_(std::move(other.tris_)), bb_(std::move(other.bb_))
  {
  }
  ~CoplanarCluster() = default;
  CoplanarCluster &operator=(const CoplanarCluster &other)
  {
    if (this != &other) {
      tris_ = other.tris_;
      bb_ = other.bb_;
    }
    return *this;
  }
  CoplanarCluster &operator=(CoplanarCluster &&other) noexcept
  {
    tris_ = std::move(other.tris_);
    bb_ = std::move(other.bb_);
    return *this;
  }

  /* Assume that caller knows this will not be a duplicate. */
  void add_tri(int t, const BoundingBox &bb)
  {
    tris_.append(t);
    bb_.combine(bb);
  }
  int tot_tri() const
  {
    return tris_.size();
  }
  int tri(int index) const
  {
    return tris_[index];
  }
  const int *begin() const
  {
    return tris_.begin();
  }
  const int *end() const
  {
    return tris_.end();
  }

  const BoundingBox &bounding_box() const
  {
    return bb_;
  }
};

/**
 * Maintains indexed set of #CoplanarCluster, with the added ability
 * to efficiently find the cluster index of any given triangle
 * (the max triangle index needs to be given in the initializer).
 * The #tri_cluster(t) function returns -1 if t is not part of any cluster.
 */
class CoplanarClusterInfo {
  Vector<CoplanarCluster> clusters_;
  Array<int> tri_cluster_;

 public:
  CoplanarClusterInfo() = default;
  explicit CoplanarClusterInfo(int numtri) : tri_cluster_(Array<int>(numtri))
  {
    tri_cluster_.fill(-1);
  }

  int tri_cluster(int t) const
  {
    BLI_assert(t < tri_cluster_.size());
    return tri_cluster_[t];
  }

  int add_cluster(CoplanarCluster cl)
  {
    int c_index = clusters_.append_and_get_index(cl);
    for (int t : cl) {
      BLI_assert(t < tri_cluster_.size());
      tri_cluster_[t] = c_index;
    }
    return c_index;
  }

  int tot_cluster() const
  {
    return clusters_.size();
  }

  const CoplanarCluster *begin()
  {
    return clusters_.begin();
  }

  const CoplanarCluster *end()
  {
    return clusters_.end();
  }

  IndexRange index_range() const
  {
    return clusters_.index_range();
  }

  const CoplanarCluster &cluster(int index) const
  {
    BLI_assert(index < clusters_.size());
    return clusters_[index];
  }
};

static std::ostream &operator<<(std::ostream &os, const CoplanarCluster &cl);

static std::ostream &operator<<(std::ostream &os, const CoplanarClusterInfo &clinfo);

enum ITT_value_kind { INONE, IPOINT, ISEGMENT, ICOPLANAR };

struct ITT_value {
  enum ITT_value_kind kind;
  mpq3 p1;      /* Only relevant for IPOINT and ISEGMENT kind. */
  mpq3 p2;      /* Only relevant for ISEGMENT kind. */
  int t_source; /* Index of the source triangle that intersected the target one. */

  ITT_value() : kind(INONE), t_source(-1)
  {
  }
  ITT_value(ITT_value_kind k) : kind(k), t_source(-1)
  {
  }
  ITT_value(ITT_value_kind k, int tsrc) : kind(k), t_source(tsrc)
  {
  }
  ITT_value(ITT_value_kind k, const mpq3 &p1) : kind(k), p1(p1), t_source(-1)
  {
  }
  ITT_value(ITT_value_kind k, const mpq3 &p1, const mpq3 &p2)
      : kind(k), p1(p1), p2(p2), t_source(-1)
  {
  }
  ITT_value(const ITT_value &other)
      : kind(other.kind), p1(other.p1), p2(other.p2), t_source(other.t_source)
  {
  }
  ITT_value(ITT_value &&other) noexcept
      : kind(other.kind),
        p1(std::move(other.p1)),
        p2(std::move(other.p2)),
        t_source(other.t_source)
  {
  }
  ~ITT_value()
  {
  }
  ITT_value &operator=(const ITT_value &other)
  {
    if (this != &other) {
      kind = other.kind;
      p1 = other.p1;
      p2 = other.p2;
      t_source = other.t_source;
    }
    return *this;
  }
  ITT_value &operator=(ITT_value &&other) noexcept
  {
    kind = other.kind;
    p1 = std::move(other.p1);
    p2 = std::move(other.p2);
    t_source = other.t_source;
    return *this;
  }
};

static std::ostream &operator<<(std::ostream &os, const ITT_value &itt);

/**
 * Project a 3d vert to a 2d one by eliding proj_axis. This does not create
 * degeneracies as long as the projection axis is one where the corresponding
 * component of the originating plane normal is non-zero.
 */
static mpq2 project_3d_to_2d(const mpq3 &p3d, int proj_axis)
{
  mpq2 p2d;
  switch (proj_axis) {
    case (0): {
      p2d[0] = p3d[1];
      p2d[1] = p3d[2];
      break;
    }
    case (1): {
      p2d[0] = p3d[0];
      p2d[1] = p3d[2];
      break;
    }
    case (2): {
      p2d[0] = p3d[0];
      p2d[1] = p3d[1];
      break;
    }
    default:
      BLI_assert(false);
  }
  return p2d;
}

/**
   Is a point in the interior of a 2d triangle or on one of its
 * edges but not either endpoint of the edge?
 * orient[pi][i] is the orientation test of the point pi against
 * the side of the triangle starting at index i.
 * Assume the triangle is non-degenerate and CCW-oriented.
 * Then answer is true if p is left of or on all three of triangle a's edges,
 * and strictly left of at least on of them.
 */
static bool non_trivially_2d_point_in_tri(const int orients[3][3], int pi)
{
  int p_left_01 = orients[pi][0];
  int p_left_12 = orients[pi][1];
  int p_left_20 = orients[pi][2];
  return (p_left_01 >= 0 && p_left_12 >= 0 && p_left_20 >= 0 &&
          (p_left_01 + p_left_12 + p_left_20) >= 2);
}

/**
 * Given orients as defined in non_trivially_2d_intersect, do the triangles
 * overlap in a "hex" pattern? That is, the overlap region is a hexagon, which
 * one gets by having, each point of one triangle being strictly right-of one
 * edge of the other and strictly left of the other two edges; and vice versa.
 * In addition, it must not be the case that all of the points of one triangle
 * are totally to one side of one edge of the other triangle, and vice versa.
 */
static bool non_trivially_2d_hex_overlap(int orients[2][3][3])
{
  for (int ab = 0; ab < 2; ++ab) {
    for (int i = 0; i < 3; ++i) {
      bool ok = orients[ab][i][0] + orients[ab][i][1] + orients[ab][i][2] == 1 &&
                orients[ab][i][0] != 0 && orients[ab][i][1] != 0 && orients[i][2] != 0;
      if (!ok) {
        return false;
      }
      int s = orients[ab][0][i] + orients[ab][1][i] + orients[ab][2][i];
      if (s == 3 || s == -3) {
        return false;
      }
    }
  }
  return true;
}

/**
 * Given orients as defined in non_trivially_2d_intersect, do the triangles
 * have one shared edge in a "folded-over" configuration?
 * As well as a shared edge, the third vertex of one triangle needs to be
 * right-of one and left-of the other two edges of the other triangle.
 */
static bool non_trivially_2d_shared_edge_overlap(int orients[2][3][3],
                                                 const mpq2 *a[3],
                                                 const mpq2 *b[3])
{
  for (int i = 0; i < 3; ++i) {
    int in = (i + 1) % 3;
    int inn = (i + 2) % 3;
    for (int j = 0; j < 3; ++j) {
      int jn = (j + 1) % 3;
      int jnn = (j + 2) % 3;
      if (*a[i] == *b[j] && *a[in] == *b[jn]) {
        /* Edge from a[i] is shared with edge from b[j]. */
        /* See if a[inn] is right-of or on one of the other edges of b.
         * If it is on, then it has to be right-of or left-of the shared edge,
         * depending on which edge it is. */
        if (orients[0][inn][jn] < 0 || orients[0][inn][jnn] < 0) {
          return true;
        }
        if (orients[0][inn][jn] == 0 && orients[0][inn][j] == 1) {
          return true;
        }
        if (orients[0][inn][jnn] == 0 && orients[0][inn][j] == -1) {
          return true;
        }
        /* Similarly for `b[jnn]`. */
        if (orients[1][jnn][in] < 0 || orients[1][jnn][inn] < 0) {
          return true;
        }
        if (orients[1][jnn][in] == 0 && orients[1][jnn][i] == 1) {
          return true;
        }
        if (orients[1][jnn][inn] == 0 && orients[1][jnn][i] == -1) {
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * Are the triangles the same, perhaps with some permutation of vertices?
 */
static bool same_triangles(const mpq2 *a[3], const mpq2 *b[3])
{
  for (int i = 0; i < 3; ++i) {
    if (a[0] == b[i] && a[1] == b[(i + 1) % 3] && a[2] == b[(i + 2) % 3]) {
      return true;
    }
  }
  return false;
}

/**
 * Do 2d triangles (a[0], a[1], a[2]) and (b[0], b[1], b2[2]) intersect at more than just shared
 * vertices or a shared edge? This is true if any point of one triangle is non-trivially inside the
 * other. NO: that isn't quite sufficient: there is also the case where the verts are all mutually
 * outside the other's triangle, but there is a hexagonal overlap region where they overlap.
 */
static bool non_trivially_2d_intersect(const mpq2 *a[3], const mpq2 *b[3])
{
  /* TODO: Could experiment with trying bounding box tests before these.
   * TODO: Find a less expensive way than 18 orient tests to do this. */

  /* `orients[0][ai][bi]` is orient of point `a[ai]` compared to segment starting at `b[bi]`.
   * `orients[1][bi][ai]` is orient of point `b[bi]` compared to segment starting at `a[ai]`. */
  int orients[2][3][3];
  for (int ab = 0; ab < 2; ++ab) {
    for (int ai = 0; ai < 3; ++ai) {
      for (int bi = 0; bi < 3; ++bi) {
        if (ab == 0) {
          orients[0][ai][bi] = orient2d(*b[bi], *b[(bi + 1) % 3], *a[ai]);
        }
        else {
          orients[1][bi][ai] = orient2d(*a[ai], *a[(ai + 1) % 3], *b[bi]);
        }
      }
    }
  }
  return non_trivially_2d_point_in_tri(orients[0], 0) ||
         non_trivially_2d_point_in_tri(orients[0], 1) ||
         non_trivially_2d_point_in_tri(orients[0], 2) ||
         non_trivially_2d_point_in_tri(orients[1], 0) ||
         non_trivially_2d_point_in_tri(orients[1], 1) ||
         non_trivially_2d_point_in_tri(orients[1], 2) || non_trivially_2d_hex_overlap(orients) ||
         non_trivially_2d_shared_edge_overlap(orients, a, b) || same_triangles(a, b);
  return true;
}

/**
 * Does triangle t in tm non-trivially non-co-planar intersect any triangle
 * in `CoplanarCluster cl`? Assume t is known to be in the same plane as all
 * the triangles in cl, and that proj_axis is a good axis to project down
 * to solve this problem in 2d.
 */
static bool non_trivially_coplanar_intersects(const IMesh &tm,
                                              int t,
                                              const CoplanarCluster &cl,
                                              int proj_axis,
                                              const Map<std::pair<int, int>, ITT_value> &itt_map)
{
  const Face &tri = *tm.face(t);
  mpq2 v0 = project_3d_to_2d(tri[0]->co_exact, proj_axis);
  mpq2 v1 = project_3d_to_2d(tri[1]->co_exact, proj_axis);
  mpq2 v2 = project_3d_to_2d(tri[2]->co_exact, proj_axis);
  if (orient2d(v0, v1, v2) != 1) {
    mpq2 tmp = v1;
    v1 = v2;
    v2 = tmp;
  }
  for (const int cl_t : cl) {
    if (!itt_map.contains(std::pair<int, int>(t, cl_t)) &&
        !itt_map.contains(std::pair<int, int>(cl_t, t))) {
      continue;
    }
    const Face &cl_tri = *tm.face(cl_t);
    mpq2 ctv0 = project_3d_to_2d(cl_tri[0]->co_exact, proj_axis);
    mpq2 ctv1 = project_3d_to_2d(cl_tri[1]->co_exact, proj_axis);
    mpq2 ctv2 = project_3d_to_2d(cl_tri[2]->co_exact, proj_axis);
    if (orient2d(ctv0, ctv1, ctv2) != 1) {
      mpq2 tmp = ctv1;
      ctv1 = ctv2;
      ctv2 = tmp;
    }
    const mpq2 *v[] = {&v0, &v1, &v2};
    const mpq2 *ctv[] = {&ctv0, &ctv1, &ctv2};
    if (non_trivially_2d_intersect(v, ctv)) {
      return true;
    }
  }
  return false;
}

/* Keeping this code for a while, but for now, almost all
 * trivial intersects are found before calling intersect_tri_tri now.
 */
#  if 0
/**
 * Do tri1 and tri2 intersect at all, and if so, is the intersection
 * something other than a common vertex or a common edge?
 * The \a itt value is the result of calling intersect_tri_tri on tri1, tri2.
 */
static bool non_trivial_intersect(const ITT_value &itt, const Face * tri1, const Face * tri2)
{
  if (itt.kind == INONE) {
    return false;
  }
  const Face * tris[2] = {tri1, tri2};
  if (itt.kind == IPOINT) {
    bool has_p_as_vert[2] {false, false};
    for (int i = 0; i < 2; ++i) {
      for (const Vert * v : *tris[i]) {
        if (itt.p1 == v->co_exact) {
          has_p_as_vert[i] = true;
          break;
        }
      }
    }
    return !(has_p_as_vert[0] && has_p_as_vert[1]);
  }
  if (itt.kind == ISEGMENT) {
    bool has_seg_as_edge[2] = {false, false};
    for (int i = 0; i < 2; ++i) {
      const Face &t = *tris[i];
      for (int pos : t.index_range()) {
        int nextpos = t.next_pos(pos);
        if ((itt.p1 == t[pos]->co_exact && itt.p2 == t[nextpos]->co_exact) ||
            (itt.p2 == t[pos]->co_exact && itt.p1 == t[nextpos]->co_exact)) {
          has_seg_as_edge[i] = true;
          break;
        }
      }
    }
    return !(has_seg_as_edge[0] && has_seg_as_edge[1]);
  }
  BLI_assert(itt.kind == ICOPLANAR);
  /* TODO: refactor this common code with code above. */
  int proj_axis = mpq3::dominant_axis(tri1->plane.norm_exact);
  mpq2 tri_2d[2][3];
  for (int i = 0; i < 2; ++i) {
    mpq2 v0 = project_3d_to_2d((*tris[i])[0]->co_exact, proj_axis);
    mpq2 v1 = project_3d_to_2d((*tris[i])[1]->co_exact, proj_axis);
    mpq2 v2 = project_3d_to_2d((*tris[i])[2]->co_exact, proj_axis);
    if (mpq2::orient2d(v0, v1, v2) != 1) {
      mpq2 tmp = v1;
      v1 = v2;
      v2 = tmp;
    }
    tri_2d[i][0] = v0;
    tri_2d[i][1] = v1;
    tri_2d[i][2] = v2;
  }
  const mpq2 *va[] = {&tri_2d[0][0], &tri_2d[0][1], &tri_2d[0][2]};
  const mpq2 *vb[] = {&tri_2d[1][0], &tri_2d[1][1], &tri_2d[1][2]};
  return non_trivially_2d_intersect(va, vb);
}
#  endif

/**
 * The sup and index functions are defined in the paper:
 * EXACT GEOMETRIC COMPUTATION USING CASCADING, by
 * Burnikel, Funke, and Seel. They are used to find absolute
 * bounds on the error due to doing a calculation in double
 * instead of exactly. For calculations involving only +, -, and *,
 * the supremum is the same function except using absolute values
 * on inputs and using + instead of -.
 * The index function follows these rules:
 *    index(x op y) = 1 + max(index(x), index(y)) for op + or -
 *    index(x * y)  = 1 + index(x) + index(y)
 *    index(x) = 0 if input x can be represented exactly as a double
 *    index(x) = 1 otherwise.
 *
 * With these rules in place, we know an absolute error bound:
 *
 *     |E_exact - E| <= supremum(E) * index(E) * DBL_EPSILON
 *
 * where E_exact is what would have been the exact value of the
 * expression and E is the one calculated with doubles.
 *
 * So the sign of E is the same as the sign of E_exact if
 *    |E| > supremum(E) * index(E) * DBL_EPSILON
 *
 * Note: a possible speedup would be to have a simple function
 * that calculates the error bound if one knows that all values
 * are less than some global maximum - most of the function would
 * be calculated ahead of time. The global max could be passed
 * from above.
 */
static double supremum_dot_cross(const double3 &a, const double3 &b)
{
  double3 abs_a = double3::abs(a);
  double3 abs_b = double3::abs(b);
  double3 c;
  /* This is dot(cross(a, b), cross(a,b)) but using absolute values for a and b
   * and always using + when operation is + or -. */
  c[0] = abs_a[1] * abs_b[2] + abs_a[2] * abs_b[1];
  c[1] = abs_a[2] * abs_b[0] + abs_a[0] * abs_b[2];
  c[2] = abs_a[0] * abs_b[1] + abs_a[1] * abs_b[0];
  return double3::dot(c, c);
}

/**
 * Used with supremum to get error bound. See Burnikel et al paper.
 * index_plane_coord is the index of a plane coordinate calculated
 * for a triangle using the usual formula, assuming the input
 * coordinates have index 1.
 * index_cross is the index of each coordinate of the cross product.
 * It is actually 2 + 2 * (max index of input coords).
 * index_dot_cross is the index of the dot product of two cross products.
 * It is actually 7 + 4 * (max index of input coords)
 */
constexpr int index_dot_cross = 11;

static double supremum_dot(const double3 &a, const double3 &b)
{
  double3 abs_a = double3::abs(a);
  double3 abs_b = double3::abs(b);
  return double3::dot(abs_a, abs_b);
}

/* Actually index_dot = 3 + 2 * (max index of input coordinates). */
/* The index of dot when inputs are plane_coords with index 1 is much higher.
 * Plane coords have index 6.
 */
constexpr int index_dot_plane_coords = 15;

static double supremum_orient3d(const double3 &a,
                                const double3 &b,
                                const double3 &c,
                                const double3 &d)
{
  double3 abs_a = double3::abs(a);
  double3 abs_b = double3::abs(b);
  double3 abs_c = double3::abs(c);
  double3 abs_d = double3::abs(d);
  double adx = abs_a[0] + abs_d[0];
  double bdx = abs_b[0] + abs_d[0];
  double cdx = abs_c[0] + abs_d[0];
  double ady = abs_a[1] + abs_d[1];
  double bdy = abs_b[1] + abs_d[1];
  double cdy = abs_c[1] + abs_d[1];
  double adz = abs_a[2] + abs_d[2];
  double bdz = abs_b[2] + abs_d[2];
  double cdz = abs_c[2] + abs_d[2];

  double bdxcdy = bdx * cdy;
  double cdxbdy = cdx * bdy;

  double cdxady = cdx * ady;
  double adxcdy = adx * cdy;

  double adxbdy = adx * bdy;
  double bdxady = bdx * ady;

  double det = adz * (bdxcdy + cdxbdy) + bdz * (cdxady + adxcdy) + cdz * (adxbdy + bdxady);
  return det;
}

/** Actually index_orient3d = 10 + 4 * (max degree of input coordinates) */
constexpr int index_orient3d = 14;

/**
 * Return the approximate orient3d of the four double3's, with
 * the guarantee that if the value is -1 or 1 then the underlying
 * mpq3 test would also have returned that value.
 * When the return value is 0, we are not sure of the sign.
 */
static int filter_orient3d(const double3 &a, const double3 &b, const double3 &c, const double3 &d)
{
  double o3dfast = orient3d_fast(a, b, c, d);
  if (o3dfast == 0.0) {
    return 0;
  }
  double err_bound = supremum_orient3d(a, b, c, d) * index_orient3d * DBL_EPSILON;
  if (fabs(o3dfast) > err_bound) {
    return o3dfast > 0.0 ? 1 : -1;
  }
  return 0;
}

/**
 * Return the approximate orient3d of the triangle plane points and v, with
 * the guarantee that if the value is -1 or 1 then the underlying
 * mpq3 test would also have returned that value.
 * When the return value is 0, we are not sure of the sign.
 */
static int filter_tri_plane_vert_orient3d(const Face &tri, const Vert *v)
{
  return filter_orient3d(tri[0]->co, tri[1]->co, tri[2]->co, v->co);
}

/**
 * Are vectors a and b parallel or nearly parallel?
 * This routine should only return false if we are certain
 * that they are not parallel, taking into account the
 * possible numeric errors and input value approximation.
 */
static bool near_parallel_vecs(const double3 &a, const double3 &b)
{
  double3 cr = double3::cross_high_precision(a, b);
  double cr_len_sq = cr.length_squared();
  if (cr_len_sq == 0.0) {
    return true;
  }
  double err_bound = supremum_dot_cross(a, b) * index_dot_cross * DBL_EPSILON;
  if (cr_len_sq > err_bound) {
    return false;
  }
  return true;
}

/**
 * Return true if we are sure that dot(a,b) > 0, taking into
 * account the error bounds due to numeric errors and input value
 * approximation.
 */
static bool dot_must_be_positive(const double3 &a, const double3 &b)
{
  double d = double3::dot(a, b);
  if (d <= 0.0) {
    return false;
  }
  double err_bound = supremum_dot(a, b) * index_dot_plane_coords * DBL_EPSILON;
  if (d > err_bound) {
    return true;
  }
  return false;
}

/**
 * Return the approximate side of point p on a plane with normal plane_no and point plane_p.
 * The answer will be 1 if p is definitely above the plane, -1 if it is definitely below.
 * If the answer is 0, we are unsure about which side of the plane (or if it is on the plane).
 * In exact arithmetic, the answer is just `sgn(dot(p - plane_p, plane_no))`.
 *
 * The plane_no input is constructed, so has a higher index.
 */
constexpr int index_plane_side = 3 + 2 * index_dot_plane_coords;

static int filter_plane_side(const double3 &p,
                             const double3 &plane_p,
                             const double3 &plane_no,
                             const double3 &abs_p,
                             const double3 &abs_plane_p,
                             const double3 &abs_plane_no)
{
  double d = double3::dot(p - plane_p, plane_no);
  if (d == 0.0) {
    return 0;
  }
  double supremum = double3::dot(abs_p + abs_plane_p, abs_plane_no);
  double err_bound = supremum * index_plane_side * DBL_EPSILON;
  if (d > err_bound) {
    return d > 0 ? 1 : -1;
  }
  return 0;
}

/**
 * A fast, non-exhaustive test for non_trivial intersection.
 * If this returns false then we are sure that tri1 and tri2
 * do not intersect. If it returns true, they may or may not
 * non-trivially intersect.
 * We assume that bounding box overlap tests have already been
 * done, so don't repeat those here. This routine is checking
 * for the very common cases (when doing mesh self-intersect)
 * where triangles share an edge or a vertex, but don't
 * otherwise intersect.
 */
static bool may_non_trivially_intersect(Face *t1, Face *t2)
{
  Face &tri1 = *t1;
  Face &tri2 = *t2;
  BLI_assert(t1->plane_populated() && t2->plane_populated());
  Face::FacePos share1_pos[3];
  Face::FacePos share2_pos[3];
  int n_shared = 0;
  for (Face::FacePos p1 = 0; p1 < 3; ++p1) {
    const Vert *v1 = tri1[p1];
    for (Face::FacePos p2 = 0; p2 < 3; ++p2) {
      const Vert *v2 = tri2[p2];
      if (v1 == v2) {
        share1_pos[n_shared] = p1;
        share2_pos[n_shared] = p2;
        ++n_shared;
      }
    }
  }
  if (n_shared == 2) {
    /* t1 and t2 share an entire edge.
     * If their normals are not parallel, they cannot non-trivially intersect. */
    if (!near_parallel_vecs(tri1.plane->norm, tri2.plane->norm)) {
      return false;
    }
    /* The normals are parallel or nearly parallel.
     * If the normals are in the same direction and the edges have opposite
     * directions in the two triangles, they cannot non-trivially intersect. */
    bool erev1 = tri1.prev_pos(share1_pos[0]) == share1_pos[1];
    bool erev2 = tri2.prev_pos(share2_pos[0]) == share2_pos[1];
    if (erev1 != erev2) {
      if (dot_must_be_positive(tri1.plane->norm, tri2.plane->norm)) {
        return false;
      }
    }
  }
  else if (n_shared == 1) {
    /* t1 and t2 share a vertex, but not an entire edge.
     * If the two non-shared verts of t2 are both on the same
     * side of tri1's plane, then they cannot non-trivially intersect.
     * (There are some other cases that could be caught here but
     * they are more expensive to check). */
    Face::FacePos p = share2_pos[0];
    const Vert *v2a = p == 0 ? tri2[1] : tri2[0];
    const Vert *v2b = (p == 0 || p == 1) ? tri2[2] : tri2[1];
    int o1 = filter_tri_plane_vert_orient3d(tri1, v2a);
    int o2 = filter_tri_plane_vert_orient3d(tri1, v2b);
    if (o1 == o2 && o1 != 0) {
      return false;
    }
    p = share1_pos[0];
    const Vert *v1a = p == 0 ? tri1[1] : tri1[0];
    const Vert *v1b = (p == 0 || p == 1) ? tri1[2] : tri1[1];
    o1 = filter_tri_plane_vert_orient3d(tri2, v1a);
    o2 = filter_tri_plane_vert_orient3d(tri2, v1b);
    if (o1 == o2 && o1 != 0) {
      return false;
    }
  }
  /* We weren't able to prove that any intersection is trivial. */
  return true;
}

/*
 * interesect_tri_tri and helper functions.
 * This code uses the algorithm of Guigue and Devillers, as described
 * in "Faster Triangle-Triangle Intersection Tests".
 * Adapted from github code by Eric Haines:
 * github.com/erich666/jgt-code/tree/master/Volume_08/Number_1/Guigue2003
 */

/**
 * Return the point on ab where the plane with normal n containing point c intersects it.
 * Assumes ab is not perpendicular to n.
 * This works because the ratio of the projections of ab and ac onto n is the same as
 * the ratio along the line ab of the intersection point to the whole of ab.
 */
static inline mpq3 tti_interp(const mpq3 &a, const mpq3 &b, const mpq3 &c, const mpq3 &n)
{
  mpq3 ab = a - b;
  mpq_class den = mpq3::dot(ab, n);
  BLI_assert(den != 0);
  mpq_class alpha = mpq3::dot(a - c, n) / den;
  return a - alpha * ab;
}

/**
 * Return +1, 0, -1 as a + ad is above, on, or below the oriented plane containing a, b, c in CCW
 * order. This is the same as -oriented(a, b, c, a + ad), but uses fewer arithmetic operations.
 * TODO: change arguments to `const Vert *` and use floating filters.
 */
static inline int tti_above(const mpq3 &a, const mpq3 &b, const mpq3 &c, const mpq3 &ad)
{
  mpq3 n = mpq3::cross(b - a, c - a);
  return sgn(mpq3::dot(ad, n));
}

/**
 * Given that triangles (p1, q1, r1) and (p2, q2, r2) are in canonical order,
 * use the classification chart in the Guigue and Devillers paper to find out
 * how the intervals [i,j] and [k,l] overlap, where [i,j] is where p1r1 and p1q1
 * intersect the plane-plane intersection line, L, and [k,l] is where p2q2 and p2r2
 * intersect L. By the canonicalization, those segments intersect L exactly once.
 * Canonicalization has made it so that for p1, q1, r1, either:
 * (a)) p1 is off the second triangle's plane and both q1 and r1 are either
 *   on the plane or on the other side of it from p1;  or
 * (b) p1 is on the plane both q1 and r1 are on the same side
 *   of the plane and at least one of q1 and r1 are off the plane.
 * Similarly for p2, q2, r2 with respect to the first triangle's plane.
 */
static ITT_value itt_canon2(const mpq3 &p1,
                            const mpq3 &q1,
                            const mpq3 &r1,
                            const mpq3 &p2,
                            const mpq3 &q2,
                            const mpq3 &r2,
                            const mpq3 &n1,
                            const mpq3 &n2)
{
  constexpr int dbg_level = 0;
  if (dbg_level > 0) {
    std::cout << "\ntri_tri_intersect_canon:\n";
    std::cout << "p1=" << p1 << " q1=" << q1 << " r1=" << r1 << "\n";
    std::cout << "p2=" << p2 << " q2=" << q2 << " r2=" << r2 << "\n";
    std::cout << "n1=" << n1 << " n2=" << n2 << "\n";
    std::cout << "approximate values:\n";
    std::cout << "p1=(" << p1[0].get_d() << "," << p1[1].get_d() << "," << p1[2].get_d() << ")\n";
    std::cout << "q1=(" << q1[0].get_d() << "," << q1[1].get_d() << "," << q1[2].get_d() << ")\n";
    std::cout << "r1=(" << r1[0].get_d() << "," << r1[1].get_d() << "," << r1[2].get_d() << ")\n";
    std::cout << "p2=(" << p2[0].get_d() << "," << p2[1].get_d() << "," << p2[2].get_d() << ")\n";
    std::cout << "q2=(" << q2[0].get_d() << "," << q2[1].get_d() << "," << q2[2].get_d() << ")\n";
    std::cout << "r2=(" << r2[0].get_d() << "," << r2[1].get_d() << "," << r2[2].get_d() << ")\n";
    std::cout << "n1=(" << n1[0].get_d() << "," << n1[1].get_d() << "," << n1[2].get_d() << ")\n";
    std::cout << "n2=(" << n2[0].get_d() << "," << n2[1].get_d() << "," << n2[2].get_d() << ")\n";
  }
  mpq3 p1p2 = p2 - p1;
  mpq3 intersect_1;
  mpq3 intersect_2;
  bool no_overlap = false;
  /* Top test in classification tree. */
  if (tti_above(p1, q1, r2, p1p2) > 0) {
    /* Middle right test in classification tree. */
    if (tti_above(p1, r1, r2, p1p2) <= 0) {
      /* Bottom right test in classification tree. */
      if (tti_above(p1, r1, q2, p1p2) > 0) {
        /* Overlap is [k [i l] j]. */
        if (dbg_level > 0) {
          std::cout << "overlap [k [i l] j]\n";
        }
        /* i is intersect with p1r1. l is intersect with p2r2. */
        intersect_1 = tti_interp(p1, r1, p2, n2);
        intersect_2 = tti_interp(p2, r2, p1, n1);
      }
      else {
        /* Overlap is [i [k l] j]. */
        if (dbg_level > 0) {
          std::cout << "overlap [i [k l] j]\n";
        }
        /* k is intersect with p2q2. l is intersect is p2r2. */
        intersect_1 = tti_interp(p2, q2, p1, n1);
        intersect_2 = tti_interp(p2, r2, p1, n1);
      }
    }
    else {
      /* No overlap: [k l] [i j]. */
      if (dbg_level > 0) {
        std::cout << "no overlap: [k l] [i j]\n";
      }
      no_overlap = true;
    }
  }
  else {
    /* Middle left test in classification tree. */
    if (tti_above(p1, q1, q2, p1p2) < 0) {
      /* No overlap: [i j] [k l]. */
      if (dbg_level > 0) {
        std::cout << "no overlap: [i j] [k l]\n";
      }
      no_overlap = true;
    }
    else {
      /* Bottom left test in classification tree. */
      if (tti_above(p1, r1, q2, p1p2) >= 0) {
        /* Overlap is [k [i j] l]. */
        if (dbg_level > 0) {
          std::cout << "overlap [k [i j] l]\n";
        }
        /* i is intersect with p1r1. j is intersect with p1q1. */
        intersect_1 = tti_interp(p1, r1, p2, n2);
        intersect_2 = tti_interp(p1, q1, p2, n2);
      }
      else {
        /* Overlap is [i [k j] l]. */
        if (dbg_level > 0) {
          std::cout << "overlap [i [k j] l]\n";
        }
        /* k is intersect with p2q2. j is intersect with p1q1. */
        intersect_1 = tti_interp(p2, q2, p1, n1);
        intersect_2 = tti_interp(p1, q1, p2, n2);
      }
    }
  }
  if (no_overlap) {
    return ITT_value(INONE);
  }
  if (intersect_1 == intersect_2) {
    if (dbg_level > 0) {
      std::cout << "single intersect: " << intersect_1 << "\n";
    }
    return ITT_value(IPOINT, intersect_1);
  }
  if (dbg_level > 0) {
    std::cout << "intersect segment: " << intersect_1 << ", " << intersect_2 << "\n";
  }
  return ITT_value(ISEGMENT, intersect_1, intersect_2);
}

/* Helper function for intersect_tri_tri. Args have been canonicalized for triangle 1. */

static ITT_value itt_canon1(const mpq3 &p1,
                            const mpq3 &q1,
                            const mpq3 &r1,
                            const mpq3 &p2,
                            const mpq3 &q2,
                            const mpq3 &r2,
                            const mpq3 &n1,
                            const mpq3 &n2,
                            int sp2,
                            int sq2,
                            int sr2)
{
  constexpr int dbg_level = 0;
  if (sp2 > 0) {
    if (sq2 > 0) {
      return itt_canon2(p1, r1, q1, r2, p2, q2, n1, n2);
    }
    if (sr2 > 0) {
      return itt_canon2(p1, r1, q1, q2, r2, p2, n1, n2);
    }
    return itt_canon2(p1, q1, r1, p2, q2, r2, n1, n2);
  }
  if (sp2 < 0) {
    if (sq2 < 0) {
      return itt_canon2(p1, q1, r1, r2, p2, q2, n1, n2);
    }
    if (sr2 < 0) {
      return itt_canon2(p1, q1, r1, q2, r2, p2, n1, n2);
    }
    return itt_canon2(p1, r1, q1, p2, q2, r2, n1, n2);
  }
  if (sq2 < 0) {
    if (sr2 >= 0) {
      return itt_canon2(p1, r1, q1, q2, r2, p2, n1, n2);
    }
    return itt_canon2(p1, q1, r1, p2, q2, r2, n1, n2);
  }
  if (sq2 > 0) {
    if (sr2 > 0) {
      return itt_canon2(p1, r1, q1, p2, q2, r2, n1, n2);
    }
    return itt_canon2(p1, q1, r1, q2, r2, p2, n1, n2);
  }
  if (sr2 > 0) {
    return itt_canon2(p1, q1, r1, r2, p2, q2, n1, n2);
  }
  if (sr2 < 0) {
    return itt_canon2(p1, r1, q1, r2, p2, q2, n1, n2);
  }
  if (dbg_level > 0) {
    std::cout << "triangles are co-planar\n";
  }
  return ITT_value(ICOPLANAR);
}

static ITT_value intersect_tri_tri(const IMesh &tm, int t1, int t2)
{
  constexpr int dbg_level = 0;
#  ifdef PERFDEBUG
  incperfcount(1); /* Intersect_tri_tri calls. */
#  endif
  const Face &tri1 = *tm.face(t1);
  const Face &tri2 = *tm.face(t2);
  BLI_assert(tri1.plane_populated() && tri2.plane_populated());
  const Vert *vp1 = tri1[0];
  const Vert *vq1 = tri1[1];
  const Vert *vr1 = tri1[2];
  const Vert *vp2 = tri2[0];
  const Vert *vq2 = tri2[1];
  const Vert *vr2 = tri2[2];
  if (dbg_level > 0) {
    std::cout << "\nINTERSECT_TRI_TRI t1=" << t1 << ", t2=" << t2 << "\n";
    std::cout << "  p1 = " << vp1 << "\n";
    std::cout << "  q1 = " << vq1 << "\n";
    std::cout << "  r1 = " << vr1 << "\n";
    std::cout << "  p2 = " << vp2 << "\n";
    std::cout << "  q2 = " << vq2 << "\n";
    std::cout << "  r2 = " << vr2 << "\n";
  }

  /* Get signs of t1's vertices' distances to plane of t2 and vice versa. */

  /* Try first getting signs with double arithmetic, with error bounds.
   * If the signs calculated in this section are not 0, they are the same
   * as what they would be using exact arithmetic. */
  const double3 &d_p1 = vp1->co;
  const double3 &d_q1 = vq1->co;
  const double3 &d_r1 = vr1->co;
  const double3 &d_p2 = vp2->co;
  const double3 &d_q2 = vq2->co;
  const double3 &d_r2 = vr2->co;
  const double3 &d_n2 = tri2.plane->norm;

  const double3 &abs_d_p1 = double3::abs(d_p1);
  const double3 &abs_d_q1 = double3::abs(d_q1);
  const double3 &abs_d_r1 = double3::abs(d_r1);
  const double3 &abs_d_r2 = double3::abs(d_r2);
  const double3 &abs_d_n2 = double3::abs(d_n2);

  int sp1 = filter_plane_side(d_p1, d_r2, d_n2, abs_d_p1, abs_d_r2, abs_d_n2);
  int sq1 = filter_plane_side(d_q1, d_r2, d_n2, abs_d_q1, abs_d_r2, abs_d_n2);
  int sr1 = filter_plane_side(d_r1, d_r2, d_n2, abs_d_r1, abs_d_r2, abs_d_n2);
  if ((sp1 > 0 && sq1 > 0 && sr1 > 0) || (sp1 < 0 && sq1 < 0 && sr1 < 0)) {
#  ifdef PERFDEBUG
    incperfcount(2); /* Tri tri intersects decided by filter plane tests. */
#  endif
    if (dbg_level > 0) {
      std::cout << "no intersection, all t1's verts above or below t2\n";
    }
    return ITT_value(INONE);
  }

  const double3 &d_n1 = tri1.plane->norm;
  const double3 &abs_d_p2 = double3::abs(d_p2);
  const double3 &abs_d_q2 = double3::abs(d_q2);
  const double3 &abs_d_n1 = double3::abs(d_n1);

  int sp2 = filter_plane_side(d_p2, d_r1, d_n1, abs_d_p2, abs_d_r1, abs_d_n1);
  int sq2 = filter_plane_side(d_q2, d_r1, d_n1, abs_d_q2, abs_d_r1, abs_d_n1);
  int sr2 = filter_plane_side(d_r2, d_r1, d_n1, abs_d_r2, abs_d_r1, abs_d_n1);
  if ((sp2 > 0 && sq2 > 0 && sr2 > 0) || (sp2 < 0 && sq2 < 0 && sr2 < 0)) {
#  ifdef PERFDEBUG
    incperfcount(2); /* Tri tri intersects decided by filter plane tests. */
#  endif
    if (dbg_level > 0) {
      std::cout << "no intersection, all t2's verts above or below t1\n";
    }
    return ITT_value(INONE);
  }

  const mpq3 &p1 = vp1->co_exact;
  const mpq3 &q1 = vq1->co_exact;
  const mpq3 &r1 = vr1->co_exact;
  const mpq3 &p2 = vp2->co_exact;
  const mpq3 &q2 = vq2->co_exact;
  const mpq3 &r2 = vr2->co_exact;

  const mpq3 &n2 = tri2.plane->norm_exact;
  if (sp1 == 0) {
    sp1 = sgn(mpq3::dot(p1 - r2, n2));
  }
  if (sq1 == 0) {
    sq1 = sgn(mpq3::dot(q1 - r2, n2));
  }
  if (sr1 == 0) {
    sr1 = sgn(mpq3::dot(r1 - r2, n2));
  }

  if (dbg_level > 1) {
    std::cout << "  sp1=" << sp1 << " sq1=" << sq1 << " sr1=" << sr1 << "\n";
  }

  if ((sp1 * sq1 > 0) && (sp1 * sr1 > 0)) {
    if (dbg_level > 0) {
      std::cout << "no intersection, all t1's verts above or below t2 (exact)\n";
    }
#  ifdef PERFDEBUG
    incperfcount(3); /* Tri tri intersects decided by exact plane tests. */
#  endif
    return ITT_value(INONE);
  }

  /* Repeat for signs of t2's vertices with respect to plane of t1. */
  const mpq3 &n1 = tri1.plane->norm_exact;
  if (sp2 == 0) {
    sp2 = sgn(mpq3::dot(p2 - r1, n1));
  }
  if (sq2 == 0) {
    sq2 = sgn(mpq3::dot(q2 - r1, n1));
  }
  if (sr2 == 0) {
    sr2 = sgn(mpq3::dot(r2 - r1, n1));
  }

  if (dbg_level > 1) {
    std::cout << "  sp2=" << sp2 << " sq2=" << sq2 << " sr2=" << sr2 << "\n";
  }

  if ((sp2 * sq2 > 0) && (sp2 * sr2 > 0)) {
    if (dbg_level > 0) {
      std::cout << "no intersection, all t2's verts above or below t1 (exact)\n";
    }
#  ifdef PERFDEBUG
    incperfcount(3); /* Tri tri intersects decided by exact plane tests. */
#  endif
    return ITT_value(INONE);
  }

  /* Do rest of the work with vertices in a canonical order, where p1 is on
   * positive side of plane and q1, r1 are not, or p1 is on the plane and
   * q1 and r1 are off the plane on the same side. */
  ITT_value ans;
  if (sp1 > 0) {
    if (sq1 > 0) {
      ans = itt_canon1(r1, p1, q1, p2, r2, q2, n1, n2, sp2, sr2, sq2);
    }
    else if (sr1 > 0) {
      ans = itt_canon1(q1, r1, p1, p2, r2, q2, n1, n2, sp2, sr2, sq2);
    }
    else {
      ans = itt_canon1(p1, q1, r1, p2, q2, r2, n1, n2, sp2, sq2, sr2);
    }
  }
  else if (sp1 < 0) {
    if (sq1 < 0) {
      ans = itt_canon1(r1, p1, q1, p2, q2, r2, n1, n2, sp2, sq2, sr2);
    }
    else if (sr1 < 0) {
      ans = itt_canon1(q1, r1, p1, p2, q2, r2, n1, n2, sp2, sq2, sr2);
    }
    else {
      ans = itt_canon1(p1, q1, r1, p2, r2, q2, n1, n2, sp2, sr2, sq2);
    }
  }
  else {
    if (sq1 < 0) {
      if (sr1 >= 0) {
        ans = itt_canon1(q1, r1, p1, p2, r2, q2, n1, n2, sp2, sr2, sq2);
      }
      else {
        ans = itt_canon1(p1, q1, r1, p2, q2, r2, n1, n2, sp2, sq2, sr2);
      }
    }
    else if (sq1 > 0) {
      if (sr1 > 0) {
        ans = itt_canon1(p1, q1, r1, p2, r2, q2, n1, n2, sp2, sr2, sq2);
      }
      else {
        ans = itt_canon1(q1, r1, p1, p2, q2, r2, n1, n2, sp2, sq2, sr2);
      }
    }
    else {
      if (sr1 > 0) {
        ans = itt_canon1(r1, p1, q1, p2, q2, r2, n1, n2, sp2, sq2, sr2);
      }
      else if (sr1 < 0) {
        ans = itt_canon1(r1, p1, q1, p2, r2, q2, n1, n2, sp2, sr2, sq2);
      }
      else {
        if (dbg_level > 0) {
          std::cout << "triangles are co-planar\n";
        }
        ans = ITT_value(ICOPLANAR);
      }
    }
  }
  if (ans.kind == ICOPLANAR) {
    ans.t_source = t2;
  }

#  ifdef PERFDEBUG
  if (ans.kind != INONE) {
    incperfcount(4);
  }
#  endif
  return ans;
}

struct CDT_data {
  const Plane *t_plane;
  Vector<mpq2> vert;
  Vector<std::pair<int, int>> edge;
  Vector<Vector<int>> face;
  /** Parallels face, gives id from input #IMesh of input face. */
  Vector<int> input_face;
  /** Parallels face, says if input face orientation is opposite. */
  Vector<bool> is_reversed;
  /** Result of running CDT on input with (vert, edge, face). */
  CDT_result<mpq_class> cdt_out;
  int proj_axis;
};

/**
 * We could de-duplicate verts here, but CDT routine will do that anyway.
 */
static int prepare_need_vert(CDT_data &cd, const mpq3 &p3d)
{
  mpq2 p2d = project_3d_to_2d(p3d, cd.proj_axis);
  int v = cd.vert.append_and_get_index(p2d);
  return v;
}

/**
 * To un-project a 2d vert that was projected along cd.proj_axis, we copy the coordinates
 * from the two axes not involved in the projection, and use the plane equation of the
 * originating 3d plane, cd.t_plane, to derive the coordinate of the projected axis.
 * The plane equation says a point p is on the plane if dot(p, plane.n()) + plane.d() == 0.
 * Assume that the projection axis is such that plane.n()[proj_axis] != 0.
 */
static mpq3 unproject_cdt_vert(const CDT_data &cd, const mpq2 &p2d)
{
  mpq3 p3d;
  BLI_assert(cd.t_plane->exact_populated());
  BLI_assert(cd.t_plane->norm_exact[cd.proj_axis] != 0);
  const mpq3 &n = cd.t_plane->norm_exact;
  const mpq_class &d = cd.t_plane->d_exact;
  switch (cd.proj_axis) {
    case (0): {
      mpq_class num = n[1] * p2d[0] + n[2] * p2d[1] + d;
      num = -num;
      p3d[0] = num / n[0];
      p3d[1] = p2d[0];
      p3d[2] = p2d[1];
      break;
    }
    case (1): {
      p3d[0] = p2d[0];
      mpq_class num = n[0] * p2d[0] + n[2] * p2d[1] + d;
      num = -num;
      p3d[1] = num / n[1];
      p3d[2] = p2d[1];
      break;
    }
    case (2): {
      p3d[0] = p2d[0];
      p3d[1] = p2d[1];
      mpq_class num = n[0] * p2d[0] + n[1] * p2d[1] + d;
      num = -num;
      p3d[2] = num / n[2];
      break;
    }
    default:
      BLI_assert(false);
  }
  return p3d;
}

static void prepare_need_edge(CDT_data &cd, const mpq3 &p1, const mpq3 &p2)
{
  int v1 = prepare_need_vert(cd, p1);
  int v2 = prepare_need_vert(cd, p2);
  cd.edge.append(std::pair<int, int>(v1, v2));
}

static void prepare_need_tri(CDT_data &cd, const IMesh &tm, int t)
{
  const Face &tri = *tm.face(t);
  int v0 = prepare_need_vert(cd, tri[0]->co_exact);
  int v1 = prepare_need_vert(cd, tri[1]->co_exact);
  int v2 = prepare_need_vert(cd, tri[2]->co_exact);
  bool rev;
  /* How to get CCW orientation of projected triangle? Note that when look down y axis
   * as opposed to x or z, the orientation of the other two axes is not right-and-up. */
  BLI_assert(cd.t_plane->exact_populated());
  if (tri.plane->norm_exact[cd.proj_axis] >= 0) {
    rev = cd.proj_axis == 1;
  }
  else {
    rev = cd.proj_axis != 1;
  }
  int cd_t = cd.face.append_and_get_index(Vector<int>());
  cd.face[cd_t].append(v0);
  if (rev) {
    cd.face[cd_t].append(v2);
    cd.face[cd_t].append(v1);
  }
  else {
    cd.face[cd_t].append(v1);
    cd.face[cd_t].append(v2);
  }
  cd.input_face.append(t);
  cd.is_reversed.append(rev);
}

static CDT_data prepare_cdt_input(const IMesh &tm, int t, const Vector<ITT_value> itts)
{
  CDT_data ans;
  BLI_assert(tm.face(t)->plane_populated());
  ans.t_plane = tm.face(t)->plane;
  BLI_assert(ans.t_plane->exact_populated());
  ans.proj_axis = mpq3::dominant_axis(ans.t_plane->norm_exact);
  prepare_need_tri(ans, tm, t);
  for (const ITT_value &itt : itts) {
    switch (itt.kind) {
      case INONE:
        break;
      case IPOINT: {
        prepare_need_vert(ans, itt.p1);
        break;
      }
      case ISEGMENT: {
        prepare_need_edge(ans, itt.p1, itt.p2);
        break;
      }
      case ICOPLANAR: {
        prepare_need_tri(ans, tm, itt.t_source);
        break;
      }
    }
  }
  return ans;
}

static CDT_data prepare_cdt_input_for_cluster(const IMesh &tm,
                                              const CoplanarClusterInfo &clinfo,
                                              int c,
                                              const Vector<ITT_value> itts)
{
  CDT_data ans;
  BLI_assert(c < clinfo.tot_cluster());
  const CoplanarCluster &cl = clinfo.cluster(c);
  BLI_assert(cl.tot_tri() > 0);
  int t0 = cl.tri(0);
  BLI_assert(tm.face(t0)->plane_populated());
  ans.t_plane = tm.face(t0)->plane;
  BLI_assert(ans.t_plane->exact_populated());
  ans.proj_axis = mpq3::dominant_axis(ans.t_plane->norm_exact);
  for (const int t : cl) {
    prepare_need_tri(ans, tm, t);
  }
  for (const ITT_value &itt : itts) {
    switch (itt.kind) {
      case IPOINT: {
        prepare_need_vert(ans, itt.p1);
        break;
      }
      case ISEGMENT: {
        prepare_need_edge(ans, itt.p1, itt.p2);
        break;
      }
      default:
        break;
    }
  }
  return ans;
}

/**
 * Fills in cd.cdt_out with result of doing the cdt calculation on (vert, edge, face).
 */
static void do_cdt(CDT_data &cd)
{
  constexpr int dbg_level = 0;
  CDT_input<mpq_class> cdt_in;
  cdt_in.vert = Span<mpq2>(cd.vert);
  cdt_in.edge = Span<std::pair<int, int>>(cd.edge);
  cdt_in.face = Span<Vector<int>>(cd.face);
  if (dbg_level > 0) {
    std::cout << "CDT input\nVerts:\n";
    for (int i : cdt_in.vert.index_range()) {
      std::cout << "v" << i << ": " << cdt_in.vert[i] << "=(" << cdt_in.vert[i][0].get_d() << ","
                << cdt_in.vert[i][1].get_d() << ")\n";
    }
    std::cout << "Edges:\n";
    for (int i : cdt_in.edge.index_range()) {
      std::cout << "e" << i << ": (" << cdt_in.edge[i].first << ", " << cdt_in.edge[i].second
                << ")\n";
    }
    std::cout << "Tris\n";
    for (int f : cdt_in.face.index_range()) {
      std::cout << "f" << f << ": ";
      for (int j : cdt_in.face[f].index_range()) {
        std::cout << cdt_in.face[f][j] << " ";
      }
      std::cout << "\n";
    }
  }
  cdt_in.epsilon = 0; /* TODO: needs attention for non-exact T. */
  cd.cdt_out = blender::meshintersect::delaunay_2d_calc(cdt_in, CDT_INSIDE);
  if (dbg_level > 0) {
    std::cout << "\nCDT result\nVerts:\n";
    for (int i : cd.cdt_out.vert.index_range()) {
      std::cout << "v" << i << ": " << cd.cdt_out.vert[i] << "=(" << cd.cdt_out.vert[i][0].get_d()
                << "," << cd.cdt_out.vert[i][1].get_d() << "\n";
    }
    std::cout << "Tris\n";
    for (int f : cd.cdt_out.face.index_range()) {
      std::cout << "f" << f << ": ";
      for (int j : cd.cdt_out.face[f].index_range()) {
        std::cout << cd.cdt_out.face[f][j] << " ";
      }
      std::cout << "orig: ";
      for (int j : cd.cdt_out.face_orig[f].index_range()) {
        std::cout << cd.cdt_out.face_orig[f][j] << " ";
      }
      std::cout << "\n";
    }
    std::cout << "Edges\n";
    for (int e : cd.cdt_out.edge.index_range()) {
      std::cout << "e" << e << ": (" << cd.cdt_out.edge[e].first << ", "
                << cd.cdt_out.edge[e].second << ") ";
      std::cout << "orig: ";
      for (int j : cd.cdt_out.edge_orig[e].index_range()) {
        std::cout << cd.cdt_out.edge_orig[e][j] << " ";
      }
      std::cout << "\n";
    }
  }
}

static int get_cdt_edge_orig(
    int i0, int i1, const CDT_data &cd, const IMesh &in_tm, bool *r_is_intersect)
{
  int foff = cd.cdt_out.face_edge_offset;
  *r_is_intersect = false;
  for (int e : cd.cdt_out.edge.index_range()) {
    std::pair<int, int> edge = cd.cdt_out.edge[e];
    if ((edge.first == i0 && edge.second == i1) || (edge.first == i1 && edge.second == i0)) {
      /* Pick an arbitrary orig, but not one equal to NO_INDEX, if we can help it. */
      /* TODO: if edge has origs from more than on part of the nary input,
       * then want to set *r_is_intersect to true. */
      for (int orig_index : cd.cdt_out.edge_orig[e]) {
        /* orig_index encodes the triangle and pos within the triangle of the input edge. */
        if (orig_index >= foff) {
          int in_face_index = (orig_index / foff) - 1;
          int pos = orig_index % foff;
          /* We need to retrieve the edge orig field from the Face used to populate the
           * in_face_index'th face of the CDT, at the pos'th position of the face. */
          int in_tm_face_index = cd.input_face[in_face_index];
          BLI_assert(in_tm_face_index < in_tm.face_size());
          const Face *facep = in_tm.face(in_tm_face_index);
          BLI_assert(pos < facep->size());
          bool is_rev = cd.is_reversed[in_face_index];
          int eorig = is_rev ? facep->edge_orig[2 - pos] : facep->edge_orig[pos];
          if (eorig != NO_INDEX) {
            return eorig;
          }
        }
        else {
          /* This edge came from an edge input to the CDT problem,
           * so it is an intersect edge. */
          *r_is_intersect = true;
          /* TODO: maybe there is an orig index:
           * This happens if an input edge was formed by an input face having
           * an edge that is co-planar with the cluster, while the face as a whole is not. */
          return NO_INDEX;
        }
      }
      return NO_INDEX;
    }
  }
  return NO_INDEX;
}

/**
 * Using the result of CDT in cd.cdt_out, extract an #IMesh representing the subdivision
 * of input triangle t, which should be an element of cd.input_face.
 */
static IMesh extract_subdivided_tri(const CDT_data &cd,
                                    const IMesh &in_tm,
                                    int t,
                                    IMeshArena *arena)
{
  const CDT_result<mpq_class> &cdt_out = cd.cdt_out;
  int t_in_cdt = -1;
  for (int i = 0; i < cd.input_face.size(); ++i) {
    if (cd.input_face[i] == t) {
      t_in_cdt = i;
    }
  }
  if (t_in_cdt == -1) {
    std::cout << "Could not find " << t << " in cdt input tris\n";
    BLI_assert(false);
    return IMesh();
  }
  int t_orig = in_tm.face(t)->orig;
  constexpr int inline_buf_size = 20;
  Vector<Face *, inline_buf_size> faces;
  for (int f : cdt_out.face.index_range()) {
    if (cdt_out.face_orig[f].contains(t_in_cdt)) {
      BLI_assert(cdt_out.face[f].size() == 3);
      int i0 = cdt_out.face[f][0];
      int i1 = cdt_out.face[f][1];
      int i2 = cdt_out.face[f][2];
      mpq3 v0co = unproject_cdt_vert(cd, cdt_out.vert[i0]);
      mpq3 v1co = unproject_cdt_vert(cd, cdt_out.vert[i1]);
      mpq3 v2co = unproject_cdt_vert(cd, cdt_out.vert[i2]);
      /* No need to provide an original index: if coord matches
       * an original one, then it will already be in the arena
       * with the correct orig field. */
      const Vert *v0 = arena->add_or_find_vert(v0co, NO_INDEX);
      const Vert *v1 = arena->add_or_find_vert(v1co, NO_INDEX);
      const Vert *v2 = arena->add_or_find_vert(v2co, NO_INDEX);
      Face *facep;
      bool is_isect0;
      bool is_isect1;
      bool is_isect2;
      if (cd.is_reversed[t_in_cdt]) {
        int oe0 = get_cdt_edge_orig(i0, i2, cd, in_tm, &is_isect0);
        int oe1 = get_cdt_edge_orig(i2, i1, cd, in_tm, &is_isect1);
        int oe2 = get_cdt_edge_orig(i1, i0, cd, in_tm, &is_isect2);
        facep = arena->add_face(
            {v0, v2, v1}, t_orig, {oe0, oe1, oe2}, {is_isect0, is_isect1, is_isect2});
      }
      else {
        int oe0 = get_cdt_edge_orig(i0, i1, cd, in_tm, &is_isect0);
        int oe1 = get_cdt_edge_orig(i1, i2, cd, in_tm, &is_isect1);
        int oe2 = get_cdt_edge_orig(i2, i0, cd, in_tm, &is_isect2);
        facep = arena->add_face(
            {v0, v1, v2}, t_orig, {oe0, oe1, oe2}, {is_isect0, is_isect1, is_isect2});
      }
      facep->populate_plane(false);
      faces.append(facep);
    }
  }
  return IMesh(faces);
}

static IMesh extract_single_tri(const IMesh &tm, int t)
{
  Face *f = tm.face(t);
  return IMesh({f});
}

static bool bvhtreeverlap_cmp(const BVHTreeOverlap &a, const BVHTreeOverlap &b)
{
  if (a.indexA < b.indexA) {
    return true;
  }
  if ((a.indexA == b.indexA) & (a.indexB < b.indexB)) {
    return true;
  }
  return false;
}
class TriOverlaps {
  BVHTree *tree_{nullptr};
  BVHTree *tree_b_{nullptr};
  BVHTreeOverlap *overlap_{nullptr};
  uint overlap_tot_{0};

  struct CBData {
    const IMesh &tm;
    std::function<int(int)> shape_fn;
    int nshapes;
    bool use_self;
  };

 public:
  TriOverlaps(const IMesh &tm,
              const Array<BoundingBox> &tri_bb,
              int nshapes,
              std::function<int(int)> shape_fn,
              bool use_self)
  {
    constexpr int dbg_level = 0;
    if (dbg_level > 0) {
      std::cout << "TriOverlaps construction\n";
    }
    /* Tree type is 8 => octtree; axis = 6 => using XYZ axes only. */
    tree_ = BLI_bvhtree_new(tm.face_size(), FLT_EPSILON, 8, 6);
    /* In the common case of a binary boolean and no self intersection in
     * each shape, we will use two trees and simple bounding box overlap. */
    bool two_trees_no_self = nshapes == 2 && !use_self;
    if (two_trees_no_self) {
      tree_b_ = BLI_bvhtree_new(tm.face_size(), FLT_EPSILON, 8, 6);
    }
    float bbpts[6];
    for (int t : tm.face_index_range()) {
      const BoundingBox &bb = tri_bb[t];
      copy_v3_v3(bbpts, bb.min);
      copy_v3_v3(bbpts + 3, bb.max);
      int shape = shape_fn(tm.face(t)->orig);
      if (two_trees_no_self) {
        if (shape == 0) {
          BLI_bvhtree_insert(tree_, t, bbpts, 2);
        }
        else if (shape == 1) {
          BLI_bvhtree_insert(tree_b_, t, bbpts, 2);
        }
      }
      else {
        if (shape != -1) {
          BLI_bvhtree_insert(tree_, t, bbpts, 2);
        }
      }
    }
    BLI_bvhtree_balance(tree_);
    if (two_trees_no_self) {
      BLI_bvhtree_balance(tree_b_);
      /* Don't expect a lot of trivial intersects in this case. */
      overlap_ = BLI_bvhtree_overlap(tree_, tree_b_, &overlap_tot_, NULL, NULL);
    }
    else {
      CBData cbdata{tm, shape_fn, nshapes, use_self};
      if (nshapes == 1 && use_self) {
        /* Expect a lot of trivial intersects from quads that are triangulated
         * and faces that share vertices.
         * Filter them out with a callback. */
        overlap_ = BLI_bvhtree_overlap(
            tree_, tree_, &overlap_tot_, only_nontrivial_intersects, &cbdata);
      }
      else {
        overlap_ = BLI_bvhtree_overlap(
            tree_, tree_, &overlap_tot_, only_different_shapes, &cbdata);
      }
    }
    /* The rest of the code is simpler and easier to parallelize if, in the two-trees case,
     * we repeat the overlaps with indexA and indexB reversed. It is important that
     * in the repeated part, sorting will then bring things with indexB together. */
    if (two_trees_no_self) {
      overlap_ = static_cast<BVHTreeOverlap *>(
          MEM_reallocN(overlap_, 2 * overlap_tot_ * sizeof(overlap_[0])));
      for (uint i = 0; i < overlap_tot_; ++i) {
        overlap_[overlap_tot_ + i].indexA = overlap_[i].indexB;
        overlap_[overlap_tot_ + i].indexB = overlap_[i].indexA;
      }
      overlap_tot_ += overlap_tot_;
    }
    /* Sort the overlaps to bring all the intersects with a given indexA together.  */
    std::sort(overlap_, overlap_ + overlap_tot_, bvhtreeverlap_cmp);
    if (dbg_level > 0) {
      std::cout << overlap_tot_ << " overlaps found:\n";
      for (BVHTreeOverlap ov : overlap()) {
        std::cout << "A: " << ov.indexA << ", B: " << ov.indexB << "\n";
      }
    }
  }

  ~TriOverlaps()
  {
    if (tree_) {
      BLI_bvhtree_free(tree_);
    }
    if (tree_b_) {
      BLI_bvhtree_free(tree_b_);
    }
    if (overlap_) {
      MEM_freeN(overlap_);
    }
  }

  Span<BVHTreeOverlap> overlap() const
  {
    return Span<BVHTreeOverlap>(overlap_, overlap_tot_);
  }

 private:
  static bool only_nontrivial_intersects(void *userdata,
                                         int index_a,
                                         int index_b,
                                         int UNUSED(thread))
  {
    CBData *cbdata = static_cast<CBData *>(userdata);
    return may_non_trivially_intersect(cbdata->tm.face(index_a), cbdata->tm.face(index_b));
  }

  static bool only_different_shapes(void *userdata, int index_a, int index_b, int UNUSED(thread))
  {
    CBData *cbdata = static_cast<CBData *>(userdata);
    return cbdata->tm.face(index_a)->orig != cbdata->tm.face(index_b)->orig;
  }
};

/**
 * Data needed for parallelization of #calc_overlap_itts.
 */
struct OverlapIttsData {
  Vector<std::pair<int, int>> intersect_pairs;
  Map<std::pair<int, int>, ITT_value> &itt_map;
  const IMesh &tm;
  IMeshArena *arena;

  OverlapIttsData(Map<std::pair<int, int>, ITT_value> &itt_map, const IMesh &tm, IMeshArena *arena)
      : itt_map(itt_map), tm(tm), arena(arena)
  {
  }
};

/**
 * Return a std::pair containing a and b in canonical order:
 * With a <= b.
 */
static std::pair<int, int> canon_int_pair(int a, int b)
{
  if (a > b) {
    std::swap(a, b);
  }
  return std::pair<int, int>(a, b);
}

static void calc_overlap_itts_range_func(void *__restrict userdata,
                                         const int iter,
                                         const TaskParallelTLS *__restrict UNUSED(tls))
{
  constexpr int dbg_level = 0;
  OverlapIttsData *data = static_cast<OverlapIttsData *>(userdata);
  std::pair<int, int> tri_pair = data->intersect_pairs[iter];
  int a = tri_pair.first;
  int b = tri_pair.second;
  if (dbg_level > 0) {
    std::cout << "calc_overlap_itts_range_func a=" << a << ", b=" << b << "\n";
  }
  ITT_value itt = intersect_tri_tri(data->tm, a, b);
  if (dbg_level > 0) {
    std::cout << "result of intersecting " << a << " and " << b << " = " << itt << "\n";
  }
  BLI_assert(data->itt_map.contains(tri_pair));
  data->itt_map.add_overwrite(tri_pair, itt);
}

/**
 * Fill in itt_map with the vector of ITT_values that result from intersecting the triangles in ov.
 * Use a canonical order for triangles: (a,b) where  a < b.
 */
static void calc_overlap_itts(Map<std::pair<int, int>, ITT_value> &itt_map,
                              const IMesh &tm,
                              const TriOverlaps &ov,
                              IMeshArena *arena)
{
  OverlapIttsData data(itt_map, tm, arena);
  /* Put dummy values in `itt_map` initially,
   * so map entries will exist when doing the range function.
   * This means we won't have to protect the `itt_map.add_overwrite` function with a lock. */
  for (const BVHTreeOverlap &olap : ov.overlap()) {
    std::pair<int, int> key = canon_int_pair(olap.indexA, olap.indexB);
    if (!itt_map.contains(key)) {
      itt_map.add_new(key, ITT_value());
      data.intersect_pairs.append(key);
    }
  }
  int tot_intersect_pairs = data.intersect_pairs.size();
  TaskParallelSettings settings;
  BLI_parallel_range_settings_defaults(&settings);
  settings.min_iter_per_thread = 1000;
  settings.use_threading = intersect_use_threading;
  BLI_task_parallel_range(0, tot_intersect_pairs, &data, calc_overlap_itts_range_func, &settings);
}

/**
 * Data needed for parallelization of calc_subdivided_tris.
 */
struct OverlapTriRange {
  int tri_index;
  int overlap_start;
  int len;
};
struct SubdivideTrisData {
  Array<IMesh> &r_tri_subdivided;
  const IMesh &tm;
  const Map<std::pair<int, int>, ITT_value> &itt_map;
  Span<BVHTreeOverlap> overlap;
  IMeshArena *arena;

  /* This vector gives, for each triangle in tm that has an intersection
   * we want to calculate: what the index of that triangle in tm is,
   * where it starts in the ov structure as indexA, and how many
   * overlap pairs have that same indexA (they will be continuous). */
  Vector<OverlapTriRange> overlap_tri_range;

  SubdivideTrisData(Array<IMesh> &r_tri_subdivided,
                    const IMesh &tm,
                    const Map<std::pair<int, int>, ITT_value> &itt_map,
                    Span<BVHTreeOverlap> overlap,
                    IMeshArena *arena)
      : r_tri_subdivided(r_tri_subdivided),
        tm(tm),
        itt_map(itt_map),
        overlap(overlap),
        arena(arena),
        overlap_tri_range{}
  {
  }
};

static void calc_subdivided_tri_range_func(void *__restrict userdata,
                                           const int iter,
                                           const TaskParallelTLS *__restrict UNUSED(tls))
{
  constexpr int dbg_level = 0;
  SubdivideTrisData *data = static_cast<SubdivideTrisData *>(userdata);
  OverlapTriRange &otr = data->overlap_tri_range[iter];
  int t = otr.tri_index;
  if (dbg_level > 0) {
    std::cout << "calc_subdivided_tri_range_func\nt=" << t << " start=" << otr.overlap_start
              << " len=" << otr.len << "\n";
  }
  constexpr int inline_capacity = 100;
  Vector<ITT_value, inline_capacity> itts(otr.len);
  for (int j = otr.overlap_start; j < otr.overlap_start + otr.len; ++j) {
    int t_other = data->overlap[j].indexB;
    std::pair<int, int> key = canon_int_pair(t, t_other);
    ITT_value itt;
    if (data->itt_map.contains(key)) {
      itt = data->itt_map.lookup(key);
    }
    if (itt.kind != INONE) {
      itts.append(itt);
    }
    if (dbg_level > 0) {
      std::cout << "  tri t" << t_other << "; result = " << itt << "\n";
    }
  }
  if (itts.size() > 0) {
    CDT_data cd_data = prepare_cdt_input(data->tm, t, itts);
    do_cdt(cd_data);
    data->r_tri_subdivided[t] = extract_subdivided_tri(cd_data, data->tm, t, data->arena);
    if (dbg_level > 0) {
      std::cout << "subdivide output\n" << data->r_tri_subdivided[t];
    }
  }
}

/**
 * For each triangle in tm, fill in the corresponding slot in
 * r_tri_subdivided with the result of intersecting it with
 * all the other triangles in the mesh, if it intersects any others.
 * But don't do this for triangles that are part of a cluster.
 * Also, do nothing here if the answer is just the triangle itself.
 */
static void calc_subdivided_tris(Array<IMesh> &r_tri_subdivided,
                                 const IMesh &tm,
                                 const Map<std::pair<int, int>, ITT_value> &itt_map,
                                 const CoplanarClusterInfo &clinfo,
                                 const TriOverlaps &ov,
                                 IMeshArena *arena)
{
  const int dbg_level = 0;
  if (dbg_level > 0) {
    std::cout << "\nCALC_SUBDIVIDED_TRIS\n\n";
  }
  Span<BVHTreeOverlap> overlap = ov.overlap();
  SubdivideTrisData data(r_tri_subdivided, tm, itt_map, overlap, arena);
  int overlap_tot = overlap.size();
  data.overlap_tri_range = Vector<OverlapTriRange>();
  data.overlap_tri_range.reserve(overlap_tot);
  int overlap_index = 0;
  while (overlap_index < overlap_tot) {
    int t = overlap[overlap_index].indexA;
    int i = overlap_index;
    while (i + 1 < overlap_tot && overlap[i + 1].indexA == t) {
      ++i;
    }
    /* Now overlap[overlap_index] to overlap[i] have indexA == t.
     * We only record ranges for triangles that are not in clusters,
     * because the ones in clusters are handled separately. */
    if (clinfo.tri_cluster(t) == NO_INDEX) {
      int len = i - overlap_index + 1;
      if (!(len == 1 && overlap[overlap_index].indexB == t)) {
        OverlapTriRange range = {t, overlap_index, len};
        data.overlap_tri_range.append(range);
#  ifdef PERFDEBUG
        bumpperfcount(0, len); /* Non-cluster overlaps. */
#  endif
      }
    }
    overlap_index = i + 1;
  }
  int overlap_tri_range_tot = data.overlap_tri_range.size();
  TaskParallelSettings settings;
  BLI_parallel_range_settings_defaults(&settings);
  settings.min_iter_per_thread = 50;
  settings.use_threading = intersect_use_threading;
  BLI_task_parallel_range(
      0, overlap_tri_range_tot, &data, calc_subdivided_tri_range_func, &settings);
}

/* Get first index in ov where indexA == t. Assuming sorted on indexA. */
static int find_first_overlap_index(const TriOverlaps &ov, int t)
{
  Span<BVHTreeOverlap> span = ov.overlap();
  if (span.size() == 0) {
    return -1;
  }
  BVHTreeOverlap bo{t, -1};
  const BVHTreeOverlap *p = std::lower_bound(
      span.begin(), span.end(), bo, [](const BVHTreeOverlap &o1, const BVHTreeOverlap &o2) {
        return o1.indexA < o2.indexA;
      });
  if (p != span.end()) {
    return p - span.begin();
  }
  return -1;
}

static CDT_data calc_cluster_subdivided(const CoplanarClusterInfo &clinfo,
                                        int c,
                                        const IMesh &tm,
                                        const TriOverlaps &ov,
                                        const Map<std::pair<int, int>, ITT_value> &itt_map,
                                        IMeshArena *UNUSED(arena))
{
  constexpr int dbg_level = 0;
  BLI_assert(c < clinfo.tot_cluster());
  const CoplanarCluster &cl = clinfo.cluster(c);
  /* Make a CDT input with triangles from C and intersects from other triangles in tm. */
  if (dbg_level > 0) {
    std::cout << "CALC_CLUSTER_SUBDIVIDED for cluster " << c << " = " << cl << "\n";
  }
  /* Get vector itts of all intersections of a triangle of cl with any triangle of tm not
   * in cl and not co-planar with it (for that latter, if there were an intersection,
   * it should already be in cluster cl). */
  Vector<ITT_value> itts;
  Span<BVHTreeOverlap> ovspan = ov.overlap();
  for (int t : cl) {
    if (dbg_level > 0) {
      std::cout << "find intersects with triangle " << t << " of cluster\n";
    }
    int first_i = find_first_overlap_index(ov, t);
    if (first_i == -1) {
      continue;
    }
    for (int i = first_i; i < ovspan.size() && ovspan[i].indexA == t; ++i) {
      int t_other = ovspan[i].indexB;
      if (clinfo.tri_cluster(t_other) != c) {
        if (dbg_level > 0) {
          std::cout << "use intersect(" << t << "," << t_other << "\n";
        }
        std::pair<int, int> key = canon_int_pair(t, t_other);
        if (itt_map.contains(key)) {
          ITT_value itt = itt_map.lookup(key);
          if (itt.kind != INONE && itt.kind != ICOPLANAR) {
            itts.append(itt);
            if (dbg_level > 0) {
              std::cout << "  itt = " << itt << "\n";
            }
          }
        }
      }
    }
  }
  /* Use CDT to subdivide the cluster triangles and the points and segs in itts. */
  CDT_data cd_data = prepare_cdt_input_for_cluster(tm, clinfo, c, itts);
  do_cdt(cd_data);
  return cd_data;
}

static IMesh union_tri_subdivides(const blender::Array<IMesh> &tri_subdivided)
{
  int tot_tri = 0;
  for (const IMesh &m : tri_subdivided) {
    tot_tri += m.face_size();
  }
  Array<Face *> faces(tot_tri);
  int face_index = 0;
  for (const IMesh &m : tri_subdivided) {
    for (Face *f : m.faces()) {
      faces[face_index++] = f;
    }
  }
  return IMesh(faces);
}

static CoplanarClusterInfo find_clusters(const IMesh &tm,
                                         const Array<BoundingBox> &tri_bb,
                                         const Map<std::pair<int, int>, ITT_value> &itt_map)
{
  constexpr int dbg_level = 0;
  if (dbg_level > 0) {
    std::cout << "FIND_CLUSTERS\n";
  }
  CoplanarClusterInfo ans(tm.face_size());
  /* Use a VectorSet to get stable order from run to run. */
  VectorSet<int> maybe_coplanar_tris;
  maybe_coplanar_tris.reserve(2 * itt_map.size());
  for (auto item : itt_map.items()) {
    if (item.value.kind == ICOPLANAR) {
      int t1 = item.key.first;
      int t2 = item.key.second;
      maybe_coplanar_tris.add_multiple({t1, t2});
    }
  }
  if (dbg_level > 0) {
    std::cout << "found " << maybe_coplanar_tris.size() << " possible coplanar tris\n";
  }
  if (maybe_coplanar_tris.size() == 0) {
    if (dbg_level > 0) {
      std::cout << "No possible coplanar tris, so no clusters\n";
    }
    return ans;
  }
  /* There can be more than one #CoplanarCluster per plane. Accumulate them in
   * a Vector. We will have to merge some elements of the Vector as we discover
   * triangles that form intersection bridges between two or more clusters. */
  Map<Plane, Vector<CoplanarCluster>> plane_cls;
  plane_cls.reserve(maybe_coplanar_tris.size());
  for (int t : maybe_coplanar_tris) {
    /* Use a canonical version of the plane for map index.
     * We can't just store the canonical version in the face
     * since canonicalizing loses the orientation of the normal. */
    Plane tplane = *tm.face(t)->plane;
    BLI_assert(tplane.exact_populated());
    tplane.make_canonical();
    if (dbg_level > 0) {
      std::cout << "plane for tri " << t << " = " << &tplane << "\n";
    }
    /* Assume all planes are in canonical from (see canon_plane()). */
    if (plane_cls.contains(tplane)) {
      Vector<CoplanarCluster> &curcls = plane_cls.lookup(tplane);
      if (dbg_level > 0) {
        std::cout << "already has " << curcls.size() << " clusters\n";
      }
      int proj_axis = mpq3::dominant_axis(tplane.norm_exact);
      /* Partition `curcls` into those that intersect t non-trivially, and those that don't. */
      Vector<CoplanarCluster *> int_cls;
      Vector<CoplanarCluster *> no_int_cls;
      for (CoplanarCluster &cl : curcls) {
        if (dbg_level > 1) {
          std::cout << "consider intersecting with cluster " << cl << "\n";
        }
        if (bbs_might_intersect(tri_bb[t], cl.bounding_box()) &&
            non_trivially_coplanar_intersects(tm, t, cl, proj_axis, itt_map)) {
          if (dbg_level > 1) {
            std::cout << "append to int_cls\n";
          }
          int_cls.append(&cl);
        }
        else {
          if (dbg_level > 1) {
            std::cout << "append to no_int_cls\n";
          }
          no_int_cls.append(&cl);
        }
      }
      if (int_cls.size() == 0) {
        /* t doesn't intersect any existing cluster in its plane, so make one just for it. */
        if (dbg_level > 1) {
          std::cout << "no intersecting clusters for t, make a new one\n";
        }
        curcls.append(CoplanarCluster(t, tri_bb[t]));
      }
      else if (int_cls.size() == 1) {
        /* t intersects exactly one existing cluster, so can add t to that cluster. */
        if (dbg_level > 1) {
          std::cout << "exactly one existing cluster, " << int_cls[0] << ", adding to it\n";
        }
        int_cls[0]->add_tri(t, tri_bb[t]);
      }
      else {
        /* t intersections 2 or more existing clusters: need to merge them and replace all the
         * originals with the merged one in `curcls`. */
        if (dbg_level > 1) {
          std::cout << "merging\n";
        }
        CoplanarCluster mergecl;
        mergecl.add_tri(t, tri_bb[t]);
        for (CoplanarCluster *cl : int_cls) {
          for (int t : *cl) {
            mergecl.add_tri(t, tri_bb[t]);
          }
        }
        Vector<CoplanarCluster> newvec;
        newvec.append(mergecl);
        for (CoplanarCluster *cl_no_int : no_int_cls) {
          newvec.append(*cl_no_int);
        }
        plane_cls.add_overwrite(tplane, newvec);
      }
    }
    else {
      if (dbg_level > 0) {
        std::cout << "first cluster for its plane\n";
      }
      plane_cls.add_new(tplane, Vector<CoplanarCluster>{CoplanarCluster(t, tri_bb[t])});
    }
  }
  /* Does this give deterministic order for cluster ids? I think so, since
   * hash for planes is on their values, not their addresses. */
  for (auto item : plane_cls.items()) {
    for (const CoplanarCluster &cl : item.value) {
      if (cl.tot_tri() > 1) {
        ans.add_cluster(cl);
      }
    }
  }

  return ans;
}

static bool face_is_degenerate(const Face *f)
{
  const Face &face = *f;
  const Vert *v0 = face[0];
  const Vert *v1 = face[1];
  const Vert *v2 = face[2];
  if (v0 == v1 || v0 == v2 || v1 == v2) {
    return true;
  }
  double3 da = v2->co - v0->co;
  double3 db = v2->co - v1->co;
  double3 dab = double3::cross_high_precision(da, db);
  double dab_length_squared = dab.length_squared();
  double err_bound = supremum_dot_cross(dab, dab) * index_dot_cross * DBL_EPSILON;
  if (dab_length_squared > err_bound) {
    return false;
  }
  mpq3 a = v2->co_exact - v0->co_exact;
  mpq3 b = v2->co_exact - v1->co_exact;
  mpq3 ab = mpq3::cross(a, b);
  if (ab.x == 0 && ab.y == 0 && ab.z == 0) {
    return true;
  }

  return false;
}

/* Data and functions to test triangle degeneracy in parallel. */
struct DegenData {
  const IMesh &tm;
};

struct DegenChunkData {
  bool has_degenerate_tri = false;
};

static void degenerate_range_func(void *__restrict userdata,
                                  const int iter,
                                  const TaskParallelTLS *__restrict tls)
{
  DegenData *data = static_cast<DegenData *>(userdata);
  DegenChunkData *chunk_data = static_cast<DegenChunkData *>(tls->userdata_chunk);
  const Face *f = data->tm.face(iter);
  bool is_degenerate = face_is_degenerate(f);
  chunk_data->has_degenerate_tri |= is_degenerate;
}

static void degenerate_reduce(const void *__restrict UNUSED(userdata),
                              void *__restrict chunk_join,
                              void *__restrict chunk)
{
  DegenChunkData *degen_chunk_join = static_cast<DegenChunkData *>(chunk_join);
  DegenChunkData *degen_chunk = static_cast<DegenChunkData *>(chunk);
  degen_chunk_join->has_degenerate_tri |= degen_chunk->has_degenerate_tri;
}

/* Does triangle #IMesh tm have any triangles with zero area? */
static bool has_degenerate_tris(const IMesh &tm)
{
  DegenData degen_data = {tm};
  DegenChunkData degen_chunk_data;
  TaskParallelSettings settings;
  BLI_parallel_range_settings_defaults(&settings);
  settings.userdata_chunk = &degen_chunk_data;
  settings.userdata_chunk_size = sizeof(degen_chunk_data);
  settings.func_reduce = degenerate_reduce;
  settings.min_iter_per_thread = 1000;
  settings.use_threading = intersect_use_threading;
  BLI_task_parallel_range(0, tm.face_size(), &degen_data, degenerate_range_func, &settings);
  return degen_chunk_data.has_degenerate_tri;
}

static IMesh remove_degenerate_tris(const IMesh &tm_in)
{
  IMesh ans;
  Vector<Face *> new_faces;
  new_faces.reserve(tm_in.face_size());
  for (Face *f : tm_in.faces()) {
    if (!face_is_degenerate(f)) {
      new_faces.append(f);
    }
  }
  ans.set_faces(new_faces);
  return ans;
}

/* This is the main routine for calculating the self_intersection of a triangle mesh. */
IMesh trimesh_self_intersect(const IMesh &tm_in, IMeshArena *arena)
{
  return trimesh_nary_intersect(
      tm_in, 1, [](int UNUSED(t)) { return 0; }, true, arena);
}

IMesh trimesh_nary_intersect(const IMesh &tm_in,
                             int nshapes,
                             std::function<int(int)> shape_fn,
                             bool use_self,
                             IMeshArena *arena)
{
  constexpr int dbg_level = 0;
  if (dbg_level > 0) {
    std::cout << "\nTRIMESH_NARY_INTERSECT nshapes=" << nshapes << " use_self=" << use_self
              << "\n";
    for (const Face *f : tm_in.faces()) {
      BLI_assert(f->is_tri());
      UNUSED_VARS_NDEBUG(f);
    }
    if (dbg_level > 1) {
      std::cout << "input mesh:\n" << tm_in;
      for (int t : tm_in.face_index_range()) {
        std::cout << "shape(" << t << ") = " << shape_fn(tm_in.face(t)->orig) << "\n";
      }
      write_obj_mesh(const_cast<IMesh &>(tm_in), "trimesh_input");
    }
  }
#  ifdef PERFDEBUG
  perfdata_init();
  double start_time = PIL_check_seconds_timer();
  std::cout << "trimesh_nary_intersect start\n";
#  endif
  /* Usually can use tm_in but if it has degenerate or illegal triangles,
   * then need to work on a copy of it without those triangles. */
  const IMesh *tm_clean = &tm_in;
  IMesh tm_cleaned;
  if (has_degenerate_tris(tm_in)) {
    if (dbg_level > 0) {
      std::cout << "cleaning degenerate triangles\n";
    }
    tm_cleaned = remove_degenerate_tris(tm_in);
    tm_clean = &tm_cleaned;
    if (dbg_level > 1) {
      std::cout << "cleaned input mesh:\n" << tm_cleaned;
    }
  }
  /* Temporary, while developing: populate all plane normals exactly. */
  for (Face *f : tm_clean->faces()) {
    f->populate_plane(true);
  }
#  ifdef PERFDEBUG
  double clean_time = PIL_check_seconds_timer();
  std::cout << "cleaned, time = " << clean_time - start_time << "\n";
#  endif
  Array<BoundingBox> tri_bb = calc_face_bounding_boxes(*tm_clean);
#  ifdef PERFDEBUG
  double bb_calc_time = PIL_check_seconds_timer();
  std::cout << "bbs calculated, time = " << bb_calc_time - clean_time << "\n";
#  endif
  TriOverlaps tri_ov(*tm_clean, tri_bb, nshapes, shape_fn, use_self);
#  ifdef PERFDEBUG
  double overlap_time = PIL_check_seconds_timer();
  std::cout << "intersect overlaps calculated, time = " << overlap_time - bb_calc_time << "\n";
#  endif
  /* itt_map((a,b)) will hold the intersection value resulting from intersecting
   * triangles with indices a and b, where a < b. */
  Map<std::pair<int, int>, ITT_value> itt_map;
  itt_map.reserve(tri_ov.overlap().size());
  calc_overlap_itts(itt_map, *tm_clean, tri_ov, arena);
#  ifdef PERFDEBUG
  double itt_time = PIL_check_seconds_timer();
  std::cout << "itts found, time = " << itt_time - overlap_time << "\n";
#  endif
  CoplanarClusterInfo clinfo = find_clusters(*tm_clean, tri_bb, itt_map);
  if (dbg_level > 1) {
    std::cout << clinfo;
  }
#  ifdef PERFDEBUG
  double find_cluster_time = PIL_check_seconds_timer();
  std::cout << "clusters found, time = " << find_cluster_time - itt_time << "\n";
  doperfmax(0, tm_in.face_size());
  doperfmax(1, clinfo.tot_cluster());
  doperfmax(2, tri_ov.overlap().size());
#  endif
  Array<IMesh> tri_subdivided(tm_clean->face_size());
  calc_subdivided_tris(tri_subdivided, *tm_clean, itt_map, clinfo, tri_ov, arena);
#  ifdef PERFDEBUG
  double subdivided_tris_time = PIL_check_seconds_timer();
  std::cout << "subdivided tris found, time = " << subdivided_tris_time - itt_time << "\n";
#  endif
  Array<CDT_data> cluster_subdivided(clinfo.tot_cluster());
  for (int c : clinfo.index_range()) {
    cluster_subdivided[c] = calc_cluster_subdivided(clinfo, c, *tm_clean, tri_ov, itt_map, arena);
  }
#  ifdef PERFDEBUG
  double cluster_subdivide_time = PIL_check_seconds_timer();
  std::cout << "subdivided clusters found, time = "
            << cluster_subdivide_time - subdivided_tris_time << "\n";
#  endif
  for (int t : tm_clean->face_index_range()) {
    int c = clinfo.tri_cluster(t);
    if (c != NO_INDEX) {
      BLI_assert(tri_subdivided[t].face_size() == 0);
      tri_subdivided[t] = extract_subdivided_tri(cluster_subdivided[c], *tm_clean, t, arena);
    }
    else if (tri_subdivided[t].face_size() == 0) {
      tri_subdivided[t] = extract_single_tri(*tm_clean, t);
    }
  }
#  ifdef PERFDEBUG
  double extract_time = PIL_check_seconds_timer();
  std::cout << "triangles extracted, time = " << extract_time - cluster_subdivide_time << "\n";
#  endif
  IMesh combined = union_tri_subdivides(tri_subdivided);
  if (dbg_level > 1) {
    std::cout << "TRIMESH_NARY_INTERSECT answer:\n";
    std::cout << combined;
  }
#  ifdef PERFDEBUG
  double end_time = PIL_check_seconds_timer();
  std::cout << "triangles combined, time = " << end_time - extract_time << "\n";
  std::cout << "trimesh_nary_intersect done, total time = " << end_time - start_time << "\n";
  dump_perfdata();
#  endif
  return combined;
}

static std::ostream &operator<<(std::ostream &os, const CoplanarCluster &cl)
{
  os << "cl(";
  bool first = true;
  for (const int t : cl) {
    if (first) {
      first = false;
    }
    else {
      os << ",";
    }
    os << t;
  }
  os << ")";
  return os;
}

static std::ostream &operator<<(std::ostream &os, const CoplanarClusterInfo &clinfo)
{
  os << "Coplanar Cluster Info:\n";
  for (int c : clinfo.index_range()) {
    os << c << ": " << clinfo.cluster(c) << "\n";
  }
  return os;
}

static std::ostream &operator<<(std::ostream &os, const ITT_value &itt)
{
  switch (itt.kind) {
    case INONE:
      os << "none";
      break;
    case IPOINT:
      os << "point " << itt.p1;
      break;
    case ISEGMENT:
      os << "segment " << itt.p1 << " " << itt.p2;
      break;
    case ICOPLANAR:
      os << "co-planar t" << itt.t_source;
      break;
  }
  return os;
}

/**
 * Writing the obj_mesh has the side effect of populating verts.
 */
void write_obj_mesh(IMesh &m, const std::string &objname)
{
  /* Would like to use #BKE_tempdir_base() here, but that brings in dependence on kernel library.
   * This is just for developer debugging anyway,
   * and should never be called in production Blender. */
#  ifdef _WIN_32
  const char *objdir = BLI_getenv("HOME");
#  else
  const char *objdir = "/tmp/";
#  endif
  if (m.face_size() == 0) {
    return;
  }

  std::string fname = std::string(objdir) + objname + std::string(".obj");
  std::ofstream f;
  f.open(fname);
  if (!f) {
    std::cout << "Could not open file " << fname << "\n";
    return;
  }

  if (!m.has_verts()) {
    m.populate_vert();
  }
  for (const Vert *v : m.vertices()) {
    const double3 dv = v->co;
    f << "v " << dv[0] << " " << dv[1] << " " << dv[2] << "\n";
  }
  int i = 0;
  for (const Face *face : m.faces()) {
    /* OBJ files use 1-indexing for vertices. */
    f << "f ";
    for (const Vert *v : *face) {
      int i = m.lookup_vert(v);
      BLI_assert(i != NO_INDEX);
      /* OBJ files use 1-indexing for vertices. */
      f << i + 1 << " ";
    }
    f << "\n";
    ++i;
  }
  f.close();
}

#  ifdef PERFDEBUG
struct PerfCounts {
  Vector<int> count;
  Vector<const char *> count_name;
  Vector<int> max;
  Vector<const char *> max_name;
};

static PerfCounts *perfdata = nullptr;

static void perfdata_init(void)
{
  perfdata = new PerfCounts;

  /* count 0. */
  perfdata->count.append(0);
  perfdata->count_name.append("Non-cluster overlaps");

  /* count 1. */
  perfdata->count.append(0);
  perfdata->count_name.append("intersect_tri_tri calls");

  /* count 2. */
  perfdata->count.append(0);
  perfdata->count_name.append("tri tri intersects decided by filter plane tests");

  /* count 3. */
  perfdata->count.append(0);
  perfdata->count_name.append("tri tri intersects decided by exact plane tests");

  /* count 4. */
  perfdata->count.append(0);
  perfdata->count_name.append("final non-NONE intersects");

  /* max 0. */
  perfdata->max.append(0);
  perfdata->max_name.append("total faces");

  /* max 1. */
  perfdata->max.append(0);
  perfdata->max_name.append("total clusters");

  /* max 2. */
  perfdata->max.append(0);
  perfdata->max_name.append("total overlaps");
}

static void incperfcount(int countnum)
{
  perfdata->count[countnum]++;
}

static void bumpperfcount(int countnum, int amt)
{
  perfdata->count[countnum] += amt;
}

static void doperfmax(int maxnum, int val)
{
  perfdata->max[maxnum] = max_ii(perfdata->max[maxnum], val);
}

static void dump_perfdata(void)
{
  std::cout << "\nPERFDATA\n";
  for (int i : perfdata->count.index_range()) {
    std::cout << perfdata->count_name[i] << " = " << perfdata->count[i] << "\n";
  }
  for (int i : perfdata->max.index_range()) {
    std::cout << perfdata->max_name[i] << " = " << perfdata->max[i] << "\n";
  }
  delete perfdata;
}
#  endif

};  // namespace blender::meshintersect

#endif  // WITH_GMP
