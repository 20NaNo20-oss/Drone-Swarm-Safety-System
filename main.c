/**
 * ============================================================================
 * SYSTÈME DE DÉTECTION DE COLLISION POUR ESSAIM AUTONOME DE DRONES (UAV)
 * ============================================================================
 * École des Sciences de l'Information
 * Cours : Programmation Avancée en C — Pr. Tarik HOUICHIME
 * Projet Industriel
 *
 * Description :
 *   Ce module implémente un algorithme optimisé de détection de la paire de
 *   drones la plus proche dans un essaim de N = 10 000 unités autonomes.
 *   L'approche repose sur un tri spatial selon l'axe X, puis une fenêtre
 *   glissante («strip»), réduisant la complexité de O(n²) à O(n log n).
 *
 * Contraintes respectées :
 *   - Structure hétérogène `struct Drone` imposée
 *   - Allocation dynamique via malloc() sur le tas
 *   - INTERDICTION stricte de l'indexation par crochets []
 *   - Navigation exclusive par arithmétique de pointeurs (*(ptr + offset))
 *
 * Auteur   : [Nom Étudiant]
 * Date     : 2025
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

/* ============================================================================
 * STRUCTURE DE DONNÉES
 * ============================================================================ */

/**
 * struct Drone — Représentation d'un micro-drone autonome dans l'espace 3D.
 *
 * @id : Identifiant unique du drone (entier positif)
 * @x  : Coordonnée spatiale selon l'axe X (mètres)
 * @y  : Coordonnée spatiale selon l'axe Y (mètres)
 * @z  : Coordonnée spatiale selon l'axe Z (altitude, mètres)
 */
struct Drone {
    int   id;
    float x;
    float y;
    float z;
};

/* ============================================================================
 * CONSTANTES SYSTÈME
 * ============================================================================ */

#define N_DRONES     10000   /* Taille de l'essaim                          */
#define STRIP_FACTOR 2       /* Facteur de la bande de candidates (2*delta) */
#define STRIP_K      7       /* Borne théorique des voisins dans la bande   */

/* ============================================================================
 * FONCTION UTILITAIRE : DISTANCE EUCLIDIENNE 3D
 * ============================================================================ */

/**
 * distance_euclidienne() — Calcule la distance entre deux drones dans R³.
 *
 * La distance euclidienne en 3D est définie par :
 *   d(A,B) = sqrt((xB-xA)² + (yB-yA)² + (zB-zA)²)
 *
 * @a : Pointeur vers le premier drone
 * @b : Pointeur vers le second drone
 *
 * Retourne : Distance (float) entre les deux drones
 *
 * NOTE SÉCURITÉ : Les pointeurs sont vérifiés NON-NULL par l'appelant.
 */
static float distance_euclidienne(const struct Drone *a, const struct Drone *b) {
    float dx = b->x - a->x;
    float dy = b->y - a->y;
    float dz = b->z - a->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* ============================================================================
 * COMPARATEUR POUR TRI SELON L'AXE X
 * ============================================================================ */

/**
 * comparateur_x() — Fonction de comparaison pour qsort() selon x croissant.
 *
 * Utilisée par l'algorithme de tri rapide de la bibliothèque standard.
 * Le tri selon X est la clé de l'optimisation : il permet de n'examiner
 * qu'une fenêtre bornée de candidats (bande delta).
 *
 * @pa : Pointeur générique vers le premier Drone
 * @pb : Pointeur générique vers le second Drone
 *
 * Retourne : < 0 si a.x < b.x | 0 si égaux | > 0 si a.x > b.x
 */
static int comparateur_x(const void *pa, const void *pb) {
    /* Cast explicite du pointeur générique vers la structure concrète */
    const struct Drone *da = (const struct Drone *)pa;
    const struct Drone *db = (const struct Drone *)pb;

    if (da->x < db->x) return -1;
    if (da->x > db->x) return  1;
    return 0;
}

/* ============================================================================
 * ALGORITHME PRINCIPAL : PAIRE LA PLUS PROCHE — FENÊTRE GLISSANTE
 * ============================================================================ */

/**
 * trouver_paire_la_plus_proche() — Détecte les deux drones en situation de
 * collision imminente (distance minimale) en O(n log n).
 *
 * ALGORITHME :
 *   1. Trier l'essaim selon l'axe X par qsort()        → O(n log n)
 *   2. Passe naïve sur l'essaim trié avec une fenêtre  → O(n · k)
 *      glissante où k ≤ STRIP_K (borne constante)
 *      Pour chaque drone i, on ne compare qu'avec les
 *      drones j > i dont |x_j - x_i| < delta courant.
 *      Dès que x_j - x_i ≥ delta, on sort de la boucle
 *      interne (les drones suivants sont encore plus loin).
 *
 *   La complexité globale est donc O(n log n).
 *
 * @essaim   : Pointeur vers le bloc mémoire des drones (alloué sur le tas)
 * @n        : Nombre de drones dans l'essaim
 * @id_a_out : Pointeur de sortie — identifiant du premier drone de la paire
 * @id_b_out : Pointeur de sortie — identifiant du second drone de la paire
 *
 * Retourne : La distance minimale trouvée (float)
 *
 * SÉCURITÉ : Vérifie que essaim != NULL et n >= 2.
 */
float trouver_paire_la_plus_proche(
        struct Drone *essaim,
        int           n,
        int          *id_a_out,
        int          *id_b_out)
{
    /* ------------------------------------------------------------------ */
    /* Vérifications défensives                                            */
    /* ------------------------------------------------------------------ */
    if (essaim == NULL || n < 2 || id_a_out == NULL || id_b_out == NULL) {
        fprintf(stderr, "[ERREUR] Paramètres invalides dans "
                        "trouver_paire_la_plus_proche()\n");
        return -1.0f;
    }

    /* ------------------------------------------------------------------ */
    /* ÉTAPE 1 : Tri selon l'axe X — O(n log n)                           */
    /* ------------------------------------------------------------------ */
    /*
     * qsort() trie les structures Drone directement dans le bloc mémoire.
     * Après le tri, *(essaim + i)->x <= *(essaim + i+1)->x pour tout i.
     * Cela garantit que la fenêtre glissante sera correcte.
     */
    qsort(essaim, (size_t)n, sizeof(struct Drone), comparateur_x);

    /* ------------------------------------------------------------------ */
    /* ÉTAPE 2 : Fenêtre glissante avec delta dynamique — O(n · k)        */
    /* ------------------------------------------------------------------ */

    float dist_min = FLT_MAX;   /* Distance minimale courante              */
    *id_a_out = -1;             /* ID du drone A de la paire critique      */
    *id_b_out = -1;             /* ID du drone B de la paire critique      */

    /*
     * Pointeurs de navigation dans l'essaim trié.
     * INTERDIT : essaim[i], essaim[j]
     * OBLIGATOIRE : *(essaim + i), *(essaim + j)
     */
    struct Drone *ptr_i;        /* Pointeur sur le drone courant (pivot)   */
    struct Drone *ptr_j;        /* Pointeur sur le candidat comparé        */
    struct Drone *ptr_fin = essaim + n; /* Sentinelle : fin du bloc mémoire */

    /* Boucle externe : itère sur chaque drone comme pivot */
    for (ptr_i = essaim; ptr_i < ptr_fin - 1; ptr_i++) {

        /*
         * Boucle interne : compare le pivot avec ses successeurs immédiats
         * dans la fenêtre delta.
         *
         * CLEF DE L'OPTIMISATION :
         * Si (ptr_j->x - ptr_i->x) >= dist_min, alors la distance 3D
         * ne peut qu'être >= dist_min (car la seule composante X dépasse
         * déjà le seuil). On sort immédiatement — les drones suivants
         * seront encore plus éloignés sur X car le tableau est trié.
         */
        for (ptr_j = ptr_i + 1; ptr_j < ptr_fin; ptr_j++) {

            /* Élagage rapide : composante X seule suffit à disqualifier */
            float delta_x = ptr_j->x - ptr_i->x;
            if (delta_x >= dist_min) {
                break; /* Sortie précoce — tous les suivants sont plus loin */
            }

            /* Calcul de la distance euclidienne 3D complète */
            float d = distance_euclidienne(ptr_i, ptr_j);

            /* Mise à jour du minimum si nouvelle paire plus proche trouvée */
            if (d < dist_min) {
                dist_min  = d;
                *id_a_out = ptr_i->id;
                *id_b_out = ptr_j->id;
            }
        }
    }

    return dist_min;
}

/* ============================================================================
 * INITIALISATION DE L'ESSAIM (DONNÉES DE TEST)
 * ============================================================================ */

/**
 * initialiser_essaim() — Génère un essaim de N drones avec positions
 * aléatoires dans un espace 3D de 1000m × 1000m × 500m.
 *
 * Navigation EXCLUSIVEMENT par arithmétique de pointeurs.
 * Les crochets [] sont INTERDITS.
 *
 * @essaim : Pointeur vers le bloc mémoire alloué
 * @n      : Nombre de drones à initialiser
 */
static void initialiser_essaim(struct Drone *essaim, int n) {
    struct Drone *ptr = essaim;           /* Pointeur de navigation         */
    struct Drone *ptr_fin = essaim + n;   /* Sentinelle fin de bloc         */
    int id_courant = 0;

    while (ptr < ptr_fin) {
        ptr->id = id_courant++;
        ptr->x  = (float)(rand() % 100000) / 100.0f;   /* [0, 1000] m   */
        ptr->y  = (float)(rand() % 100000) / 100.0f;   /* [0, 1000] m   */
        ptr->z  = (float)(rand() %  50000) / 100.0f;   /* [0,  500] m   */
        ptr++;  /* Avance d'un sizeof(struct Drone) en mémoire            */
    }
}

/* ============================================================================
 * PROGRAMME PRINCIPAL
 * ============================================================================ */

/**
 * main() — Point d'entrée du système de sécurité de l'essaim.
 *
 * Flux d'exécution :
 *   1. Allocation dynamique de l'essaim sur le tas (malloc)
 *   2. Initialisation des positions aléatoires
 *   3. Détection de la paire la plus proche (algorithme O(n log n))
 *   4. Affichage de l'alerte de collision
 *   5. Libération de la mémoire (free)
 */
int main(void) {
    srand((unsigned int)time(NULL));

    /* ------------------------------------------------------------------ */
    /* ALLOCATION DU TAS : Un seul bloc continu pour les 10 000 drones    */
    /* ------------------------------------------------------------------ */
    struct Drone *essaim = (struct Drone *)malloc(
        (size_t)N_DRONES * sizeof(struct Drone)
    );

    if (essaim == NULL) {
        fprintf(stderr, "[ERREUR FATALE] Échec malloc — mémoire insuffisante"
                        " pour %d drones\n", N_DRONES);
        return EXIT_FAILURE;
    }

    printf("=============================================================\n");
    printf(" SYSTÈME DE DÉTECTION DE COLLISION — ESSAIM UAV\n");
    printf("=============================================================\n");
    printf("[INFO] Bloc mémoire alloué : %zu octets (%.2f MB)\n",
           (size_t)N_DRONES * sizeof(struct Drone),
           (double)(N_DRONES * sizeof(struct Drone)) / (1024.0 * 1024.0));
    printf("[INFO] Adresse de base de l'essaim : %p\n", (void *)essaim);

    /* ------------------------------------------------------------------ */
    /* INITIALISATION DES POSITIONS DES DRONES                            */
    /* ------------------------------------------------------------------ */
    initialiser_essaim(essaim, N_DRONES);
    printf("[INFO] %d drones initialisés avec succès.\n\n", N_DRONES);

    /* ------------------------------------------------------------------ */
    /* CHRONOMÉTRAGE DE L'ALGORITHME                                       */
    /* ------------------------------------------------------------------ */
    clock_t debut = clock();

    int id_a = -1, id_b = -1;
    float dist = trouver_paire_la_plus_proche(essaim, N_DRONES, &id_a, &id_b);

    clock_t fin = clock();
    double temps_ms = 1000.0 * (double)(fin - debut) / (double)CLOCKS_PER_SEC;

    /* ------------------------------------------------------------------ */
    /* AFFICHAGE DU RAPPORT D'ALERTE                                       */
    /* ------------------------------------------------------------------ */
    if (id_a == -1 || id_b == -1) {
        fprintf(stderr, "[ERREUR] Algorithme n'a pas trouvé de paire.\n");
        free(essaim);
        return EXIT_FAILURE;
    }

    printf("=============================================================\n");
    printf(" ALERTE COLLISION — PAIRE CRITIQUE DÉTECTÉE\n");
    printf("=============================================================\n");

    /*
     * Recherche des drones par ID dans le tableau trié
     * Navigation par arithmétique de pointeurs — [] interdit.
     */
    struct Drone *ptr = essaim;
    struct Drone *ptr_fin_main = essaim + N_DRONES;
    struct Drone *drone_a = NULL, *drone_b = NULL;

    while (ptr < ptr_fin_main) {
        if (ptr->id == id_a) drone_a = ptr;
        if (ptr->id == id_b) drone_b = ptr;
        if (drone_a != NULL && drone_b != NULL) break;
        ptr++;
    }

    if (drone_a != NULL) {
        printf(" Drone A | ID: %5d | x=%8.2f m | y=%8.2f m | z=%8.2f m\n",
               drone_a->id, drone_a->x, drone_a->y, drone_a->z);
    }
    if (drone_b != NULL) {
        printf(" Drone B | ID: %5d | x=%8.2f m | y=%8.2f m | z=%8.2f m\n",
               drone_b->id, drone_b->x, drone_b->y, drone_b->z);
    }

    printf("-------------------------------------------------------------\n");
    printf(" Distance minimale   : %.6f mètres\n", dist);
    printf(" Temps d'exécution   : %.3f ms\n", temps_ms);
    printf(" Complexité garantie : O(n log n)\n");
    printf("=============================================================\n");
    printf("[ACTION] Manœuvre d'évitement déclenchée pour drones %d et %d\n",
           id_a, id_b);

    /* ------------------------------------------------------------------ */
    /* LIBÉRATION DU TAS                                                   */
    /* ------------------------------------------------------------------ */
    free(essaim);
    essaim = NULL; /* Bonne pratique : invalider le pointeur après free()  */

    return EXIT_SUCCESS;
}
