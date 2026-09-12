# Plan: Consolidation de la Documentation (Constitution, Règles et Architecture)

**Statut :** Tranches principales landées (2026-09-12), amendées par
[docs/reviews/2026-09-12-constitution-second-opinion.md](../reviews/2026-09-12-constitution-second-opinion.md).
**Objectif :** Éliminer la redondance et le bruit normatif entre les documents racine de Minilink en établissant une séparation claire et sans chevauchement entre la philosophie, les règles de code, le comportement des agents et les contrats techniques.

---

## 1. Architecture documentaire cible

Chaque document répond à **une seule question précise** pour un public clairement identifié :

| Document | Public | Rôle & Contenu unique | Ce qu'il ne contient plus |
| :--- | :--- | :--- | :--- |
| **`CONSTITUTION.md`** | Humains & Agents | **La boussole suprême :** Vision du *Unified Systems Lab*, contrat mathématique continu $\dot{x}=f(x,u,t;p)$, pureté fonctionnelle (#1), fermeture de composition continue (#2), contrat continu prioritaire (#3), promesse des deux voies (enseignement/recherche), simplicité et beauté du code. | Pas de règles de syntaxe Python, pas de commandes de linter ou de CI. |
| **`RULES.md`** | Humains & Agents | **L'échelle de revue en 7 questions :** Toutes les règles concrètes, testables et vérifiables (placement, API, textbook style, bare signatures, idiome `xp`, no `self.`, no `_` pseudo-privé, démos open-and-run). | Pas de spéculation philosophique ; pas de consignes d'interaction agent. |
| **`AGENTS.md`** | Agents IA | **Guide comportemental et workflow de l'agent :** Stub non-négociable, carte des docs, préservation des modifications utilisateur, périmètre *ask first* vs *agent-managed*, porte CI locale. Renvoie à `CONSTITUTION.md` et `RULES.md` ; n'est **pas** 100 % comportemental. | Ne répète plus le textbook style ni les rappels d'architecture. |
| **`DESIGN.md`** | Développeurs & Agents | **Contrats techniques d'implémentation :** Specs (`System`, ports, compilation JAX, §8 call chains). Longueur suit le contenu ; pas de budget ~400 lignes. | Ne répète plus la philosophie générale ni les règles de style. |
| **`ROADMAP.md`** | Maintainer & Utilisateurs | **Plan de marche et maturité :** Jalons, versions, registre TRL, contrat opérationnel des deux voies (§2, table conservée). | Ne redéfinit plus les contrats mathématiques ni l'identité produit. |

---

## 2. État d'avancement

- [x] **`CONSTITUTION.md` amendé (second opinion) :**
  - Identité *Unified Systems Lab* (pas de rebrand « Differentiable »).
  - Invariant 3 restreint à l'ensemble continu ; hybride = algèbre sœur.
  - Précédence : pureté > fermeture > contrat continu prioritaire > lisibilité > chemin unique.
  - §1.4 : seulement les NOT ; la phrase d'identité positive est dans §1.1.
  - « The Contract » (plus « Sacred »).
- [x] **`RULES.md` resserré :**
  - §1–§2 : tests vérifiables, plus d'essai recopié de la constitution.
  - §3.3 pointe vers ROADMAP §2 pour la règle d'entrée / wheel / CI.
  - §4.9 nomenclature préférée (pas un rename sweep) ; §4.10 validation au câblage, pas de `f`/`h`.
  - §6.7 Colab CPU sans plafond 60 s.
  - Absorbe le droit de code qui ne vivait que dans AGENTS (fichiers two-audience, `__main__`, native-array, float64, prototype honnête, refactor incrémental).
- [x] **`AGENTS.md` élagué :** stub non-négociable + carte des docs + workflow. Pas vidé.
- [x] **`DESIGN.md` :** §1 principes / identité / table concurrente retirés ; NumPy/JAX et §2–§8 conservés.
- [x] **`ROADMAP.md` §2 :** pointeur constitution/RULES ; **table opérationnelle conservée**.

---

## 3. Tranches landées (contraintes second opinion)

### Étape 1 : `AGENTS.md` — faite, pas 100 % comportemental
Retiré : Core directives, Textbook style, Architecture reminders.
Conservé : ouverture CONSTITUTION/RULES, stub, doc map, intro-doc / examples / imports / grep public, workflow, porte CI, revision pass.

### Étape 2 : `DESIGN.md` — faite, sans budget de lignes
Retiré : §1 Design Principles et Product identity & scope.
Conservé : NumPy/JAX, Interface Layers, Package Map, Core Object Contracts, compilation, optimization, graphics, call chains.

### Étape 3 : `ROADMAP.md` — faite, table §2 conservée
Pointeur vers CONSTITUTION §3 et RULES §3.3. Le contrat opérationnel (soft entry, wheel, CI) reste dans la table.

### Étape 4 : Validation
Docs only — pytest sauté. Ancres `DESIGN.md#product-identity--scope` remplacées.

---

## 4. Arbitrage (tranché par la second opinion)

1. **`AGENTS.md` :** pas 100 % comportemental. Stub + doc map + workflow ; le style vit dans `RULES.md`.
2. **`DESIGN.md` :** le fichier reste ; pas de cible ~400 lignes. Succès = plus de philosophie ni de style, contrats seulement.
